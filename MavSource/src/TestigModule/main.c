#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>

#include <libmavlink.h>
#include <cssl.h>

#define DEV_DIR "/dev"
#define PREFIX "ttyACM"
#define max_devices  100

#ifndef CONF_PATH
#define CONF_PATH "/tmp/mavconf.conf"
#endif

uint8_t rx_buf[MAVLINK_MAX_PACKET_LEN];
uint8_t chan = MAVLINK_COMM_0;
mavlink_status_t status;
cssl_t *serial;
bool finished = false ;

typedef struct {
    char * device_path;
    bool Validated; 
}DevValidation;

DevValidation * CurrTestingDev ;
DevValidation *device_list ;

void initDevValidation(DevValidation *dev, const char *path) {
    if (dev == NULL) return;
    dev->device_path = (char *)malloc(strlen(path) + 1);
    if (dev->device_path != NULL) {
        strcpy(dev->device_path, path);
    }
    dev->Validated = false;
}
void freeDevValidation(DevValidation *dev) {
    if (dev == NULL) return;  
    if (dev->device_path != NULL) {
        free(dev->device_path);
        dev->device_path = NULL;
    }
}
static void callback(int id, uint8_t *buf, int length)
{
    printf ("callback Triggered \n");
    mavlink_message_t msg;
    if (!(length>0)) {printf("[+]Error while parshing String");exit(EXIT_FAILURE);}
    for(int i=0;i<length;i++) {
        if (mavlink_parse_char(chan, buf[i], &msg, &status)){
            printf("found mavlink compatible message\n");
            CurrTestingDev->Validated = true;
        }
    }
}
void* handle_serial_func (void* arg) {

    DevValidation* current_dev = (DevValidation *) arg;
    CurrTestingDev = current_dev;
	char* curr_dev_name = current_dev->device_path;
    cssl_start();
    serial=cssl_open(curr_dev_name,callback,0,115200,8,0,1);
    sleep(2);
    if (CurrTestingDev->Validated){
        finished = true;
        printf("compatible mavlink router target at %s\n",current_dev->device_path);
    }
    printf("handling device exit\n");
    cssl_close(serial);
    cssl_stop();
	return NULL;
}

void collect_device_info(const char *device_name, DevValidation *device_list, int *device_count) {
    char device_path[256];
    struct stat device_stat;

    snprintf(device_path, sizeof(device_path), "%s/%s", DEV_DIR, device_name);

    if (stat(device_path, &device_stat) == -1) {
        fprintf(stderr, "Error getting stats for %s: %s\n", device_path, strerror(errno));
        return;
    }

    initDevValidation(&device_list[*device_count], device_path);

    printf("Device: %s\n", device_path);
    printf("  Device ID: %ld\n", (long) device_stat.st_dev);
    printf("  Inode: %ld\n", (long) device_stat.st_ino);
    printf("  Mode: %o\n", (unsigned int) device_stat.st_mode);
    printf("  Owner (UID): %d\n", device_stat.st_uid);
    printf("  Group (GID): %d\n", device_stat.st_gid);
    printf("  Size: %lld bytes\n", (long long) device_stat.st_size);
    printf("  Last Access: %ld\n", (long) device_stat.st_atime);
    printf("  Last Modification: %ld\n", (long) device_stat.st_mtime);
    printf("  Last Status Change: %ld\n\n", (long) device_stat.st_ctime);

    (*device_count)++;
}

void write_device_to_conf(const DevValidation *dev, const char *conf_path) {
    FILE *file = fopen(conf_path, "r+");
    if (file == NULL) {
        perror("Failed to open config file");
        return;
    }
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    rewind(file);
    char *buffer = (char *)malloc(file_size + 1);
    if (buffer == NULL) {
        perror("Memory allocation failed");
        fclose(file);
        return;
    }
    fread(buffer, 1, file_size, file);
    buffer[file_size] = '\0';
    const char *section = "[UartEndpoint alpha]";
    char *section_pos = strstr(buffer, section);
    if (section_pos == NULL) {
        fprintf(stderr, "Section %s not found in config file\n", section);
        free(buffer);
        fclose(file);
        return;
    }
    char *device_pos = strstr(section_pos, "Device =");
    if (device_pos == NULL) {
        fprintf(stderr, "Device field not found under %s\n", section);
        free(buffer);
        fclose(file);
        return;
    }
    device_pos += strlen("Device =");
    fseek(file, device_pos - buffer, SEEK_SET);
    fprintf(file, " %s\n", dev->device_path);
    free(buffer);
    fclose(file);

    printf("Device path %s successfully written to config file\n", dev->device_path);
}

int main() {
    struct dirent *entry;
    DIR *dp = opendir(DEV_DIR);
    int device_count = 0;
    device_list = (DevValidation *)malloc(max_devices * sizeof(DevValidation));

    if (device_list == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        return 1;
    }

    if (dp == NULL) {
        fprintf(stderr, "Error opening directory %s: %s\n", DEV_DIR, strerror(errno));
        free(device_list);
        return 1;
    }

    while ((entry = readdir(dp)) != NULL) {
        if (strncmp(entry->d_name, PREFIX, strlen(PREFIX)) == 0) {
            if (device_count < max_devices) {
                collect_device_info(entry->d_name, device_list, &device_count);
            } else {
                fprintf(stderr, "Max device limit reached\n");
                break;
            }
        }
    }
    
    for (int i = 0; i < device_count; i++) {
        pthread_t thread;
        if (pthread_create(&thread, NULL, handle_serial_func, &device_list[i]) != 0) {
            perror("Failed to create thread");
            return -1;
        }
        if (pthread_join(thread, NULL) != 0) {
            perror("Failed to join thread");
            return -1;
        }
        if (finished){
            goto validated_hop;
        }
    }
    validated_hop:
    for (int i = 0; i < device_count; i++) {
        if (device_list[i].Validated){
            write_device_to_conf(&device_list[i],CONF_PATH);
            break;
        }
    }
    for (int i = 0; i < device_count; i++) {
        freeDevValidation(&device_list[i]);
    }
    free(device_list); 
    closedir(dp);

    return 0;
}