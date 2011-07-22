// NI Adam GPS library
// Uses an Open Source NMEA parsing lib
// Written by MrGuy

#include <hardware/gps.h>
#include <stdio.h>
#include <android/log.h>
#include <pthread.h>
#include <time.h>
#include <sys/stat.h>
#include <string.h>
#include "nmea/nmea/nmea.h"

#define  GPS_DEBUG  1

#define  LOG_TAG  "gps_adam"
#define GPS_TTYPORT "/dev/ttyHS3"
#define MAX_NMEA_CHARS 85

#if GPS_DEBUG
#define LOGV(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGV(...) ((void)0)
#endif

static GpsInterface adamGpsInterface;
static GpsCallbacks* adamGpsCallbacks;
static pthread_t NMEAThread;
char NMEA[MAX_NMEA_CHARS];
pthread_mutex_t mutGPS = PTHREAD_MUTEX_INITIALIZER;
char gpsOn = 0;

typedef struct _argsCarrier
{
	char* 		NMEA;
	nmeaINFO*	info;

} argsCarrier;
/////////////////////////////////////////////////////////
//			GPS THREAD		       //
/////////////////////////////////////////////////////////

static GpsUtcTime getUTCTime(nmeaTIME *time) {
	GpsUtcTime ret = time->year*(31556926);
	ret += time->mon*(2629743);
	ret += time->day*(86400);
	ret += time->hour*(3600);
	ret += time->min*(60);
	ret += time->sec*(1);
	ret *= 1000; // Scale up to milliseconds
	ret += time->hsec*(10);
	return ret;
}

static void updateNMEA(void* arg) {
	argsCarrier *Args = (argsCarrier*)arg;
	nmeaINFO *info = Args->info;
	char* NMEA2 = Args->NMEA;
	LOGV("Debug GPS: %s", NMEA2);
	if (adamGpsCallbacks != NULL) {
		adamGpsCallbacks->nmea_cb(getUTCTime(&(info->utc)), NMEA2, strlen(NMEA2));
	}

	return;
}

static void updateRMC(void* arg) {
	argsCarrier *Args = (argsCarrier*)arg;
	nmeaINFO *info = Args->info;
	char* NMEA2 = Args->NMEA;
	GpsLocation newLoc;

	if (info->sig == 0) {
		LOGV("No valid fix data.");
		return;
	}

	newLoc.size = sizeof(GpsLocation);
	newLoc.flags = GPS_LOCATION_HAS_LAT_LONG;
	newLoc.latitude = info->lat;
	newLoc.longitude = info->lon;
	newLoc.timestamp = getUTCTime(&(info->utc));
	LOGV("Lat: %lf Long: %lf", info->lat, info->lon);
	if (adamGpsCallbacks != NULL) {
		adamGpsCallbacks->location_cb(&newLoc);
	}
	free(info);
	free(NMEA2);
	free(arg);
	

}

static void updateGSV(void* arg) {
	argsCarrier *Args = (argsCarrier*)arg;
	nmeaINFO *info = Args->info;
	char* NMEA2 = Args->NMEA;
	nmeaSATINFO sats = info->satinfo;
	int num = sats.inview;
	int count = 0;
	GpsSvStatus svStatus;

	LOGV("Updating %i sats", num);

	if (num == 0) {
		return;
	}


	for (count = 0; count < num; count++) {
		svStatus.sv_list[count].size = sizeof(GpsSvInfo);
		svStatus.sv_list[count].prn = sats.sat[count].id;
		svStatus.sv_list[count].snr = sats.sat[count].sig;
		svStatus.sv_list[count].elevation = sats.sat[count].elv;
		svStatus.sv_list[count].azimuth = sats.sat[count].azimuth;
	}
	
	svStatus.size = sizeof(GpsSvStatus);
	svStatus.num_svs = num; 
	svStatus.ephemeris_mask = 0;
	svStatus.almanac_mask = 0;
	svStatus.used_in_fix_mask = 0;
	if (adamGpsCallbacks != NULL) {
		adamGpsCallbacks->sv_status_cb(&svStatus);
	}
	free(info);
	free(NMEA2);
	free(arg);

}

void processNMEA() {
	argsCarrier *Args = malloc(sizeof(argsCarrier));        
	nmeaINFO *info = malloc(sizeof(nmeaINFO));
        nmeaPARSER parser;
        nmea_zero_INFO(info);
        nmea_parser_init(&parser);
	char *NMEA2;
	nmeaINFO info2;
	// Fix up the end so the NMEA parser accepts it
	int count = (int)strlen(NMEA);
	NMEA[count-1] = '\r';
	NMEA[count] = '\n';
	NMEA[count+1] = '\0';
	// Parse the data
	nmea_parse(&parser, NMEA, (int)strlen(NMEA), info);

	if (info->smask == 0) {
		//Bad data
		return;
	}
	// NOTE: Make copy of NMEA and Data for these threads - Don't want them to be overwritten by other threads
	// Have the individual case function take care of freeing them
	Args->NMEA = (char *)malloc((strlen(&NMEA[0])+1)*sizeof(char));
	strcpy(Args->NMEA, NMEA);
	Args->info = info;	
	
	//adamGpsCallbacks->create_thread_cb("adamgps-nmea", updateNMEA, Args);
	
	switch (info->smask) {
	case 1:
		//< GGA - Essential fix data which provide 3D location and accuracy data.
		free(Args->info);
		free(Args->NMEA);
		free(Args);
		break;
	case 2: 
		//< GSA - GPS receiver operating mode, SVs used for navigation, and DOP values.
		free(Args->info);
		free(Args->NMEA);
		free(Args);
		break;
	case 4: 
		//< GSV - Number of SVs in view, PRN numbers, elevation, azimuth & SNR values.
		adamGpsCallbacks->create_thread_cb("adamgps-gsv", updateGSV, Args);
		break;
	case 8: 
		//< RMC - Recommended Minimum Specific GPS/TRANSIT Data.
		adamGpsCallbacks->create_thread_cb("adamgps-loc", updateRMC, Args);
		break;
	case 16:
		//< VTG - Actual track made good and speed over ground.
		free(Args->info);
		free(Args->NMEA);
		free(Args);
		break;
	default:
		free(Args->info);
		free(Args->NMEA);
		free(Args);
		break;
	}

	//LOGV("Successful read: %i", info->smask);
	
}


static void* doGPS (void* arg) {
	FILE *gpsTTY = NULL;
	char* buffer = NULL;

        // Maximum NMEA sentence SHOULD be 80 characters  + terminators - include a small safety buffer 
	struct timespec slp;
	char go = 1;	
	slp.tv_sec = 1;
	slp.tv_nsec = 0;
	// Open the GPS port
	gpsTTY = fopen(GPS_TTYPORT, "r");
	if (gpsTTY == NULL) {
		LOGV("Failed opening TTY port: %s", GPS_TTYPORT);
		return NULL;
	}
	// Obtain mutex lock and check if we're good to go
	pthread_mutex_lock(&mutGPS);
	go = gpsOn;
	pthread_mutex_unlock(&mutGPS);

	while (go) {
		buffer = fgets(NMEA, MAX_NMEA_CHARS, gpsTTY);
		if (buffer == NULL) {
			LOGV("NMEA data read fail, sleeping for 1 sec.");
			nanosleep(&slp, NULL);
		}
		if (NMEA[0] == '$') {
			//We have a good sentance
			processNMEA();
			// Get rid of the extra LF
			fgetc(gpsTTY);
		} 
		pthread_mutex_lock(&mutGPS);
		go = gpsOn;
		pthread_mutex_unlock(&mutGPS);
	}
return NULL;
}


/////////////////////////////////////////////////////////
//		     GPS INTERFACE       	       //
/////////////////////////////////////////////////////////



static int gpslib_init(GpsCallbacks* callbacks) {
int ret = 0;
LOGV("Callbacks set");
adamGpsCallbacks = callbacks;
adamGpsCallbacks->set_capabilities_cb(0);

struct stat st;
if(stat(GPS_TTYPORT, &st) != 0) {
	ret = -1;
	LOGV("Specified tty port: %s does not exist", GPS_TTYPORT);
	goto end;
}

end:
return ret;
}

static int gpslib_start() {
LOGV("Gps start");
pthread_mutex_lock(&mutGPS);
gpsOn = 1;
pthread_mutex_unlock(&mutGPS);	
pthread_create(&NMEAThread, NULL, doGPS, NULL);
return 0;
}

static int gpslib_stop() {
LOGV("GPS stop");
pthread_mutex_lock(&mutGPS);
gpsOn = 0;
pthread_mutex_unlock(&mutGPS);
return 0;
}

static void gpslib_cleanup() {
LOGV("GPS clean");
return;
}

static int gpslib_inject_time(GpsUtcTime time, int64_t timeReference,
                         int uncertainty) {
LOGV("GPS inject time");
return 0;
}

static int gpslib_inject_location(double latitude, double longitude, float accuracy) {
LOGV("GPS inject location");
return 0;
}


static void gpslib_delete_aiding_data(GpsAidingData flags) {

}

static int gpslib_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
            uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time) {

return 0;
}


static const void* gpslib_get_extension(const char* name) {
return NULL;
}

const GpsInterface* gps__get_gps_interface(struct gps_device_t* dev) 
{
	LOGV("Gps get_interface");
	adamGpsInterface.size = sizeof(GpsInterface);
	adamGpsInterface.init = gpslib_init;
	adamGpsInterface.start = gpslib_start;
	adamGpsInterface.stop = gpslib_stop;
	adamGpsInterface.cleanup = gpslib_cleanup;
	adamGpsInterface.inject_time = gpslib_inject_time;
	adamGpsInterface.inject_location = gpslib_inject_location;
	adamGpsInterface.delete_aiding_data = gpslib_delete_aiding_data;
	adamGpsInterface.set_position_mode = gpslib_set_position_mode;
	adamGpsInterface.get_extension = gpslib_get_extension;

	return &adamGpsInterface;
}
	
	
static int open_gps(const struct hw_module_t* module, char const* name,
	struct hw_device_t** device) 
{
	struct gps_device_t *dev = malloc (sizeof(struct gps_device_t));
	memset(dev, 0 , sizeof(*dev));
	
	dev->common.tag = HARDWARE_DEVICE_TAG;
	dev->common.version = 0;
	dev->common.module = (struct hw_module_t*)module;
	dev->get_gps_interface = gps__get_gps_interface;
	
	*device = (struct hw_device_t*)dev;
	return 0;
}


static struct hw_module_methods_t gps_module_methods = {
	.open = open_gps
};

const struct hw_module_t HAL_MODULE_INFO_SYM = {
	.tag = HARDWARE_MODULE_TAG,
	.version_major = 1,
	.version_minor = 0,
	.id = GPS_HARDWARE_MODULE_ID,
	.name = "Adam GPS Module",
	.author = "MrGuy",
	.methods = &gps_module_methods,
};
