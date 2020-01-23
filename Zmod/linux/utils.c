/**
 * @file utils.c
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing util functions for linux-specific implementations.
 */

#ifdef LINUX_APP

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "utils.h"


/**
 * Check if a string starts with a prefix.
 *
 * @param name the name to match
 * @param prefix the prefix to search
 *
 * @return whether the name starts with prefix or not
 */
bool fnStartsWith(const char *name, const char *prefix) {
	return !strncmp(name, prefix, strlen(prefix));
}

/**
 * Read the first 4 bytes of a file into an uint32_t.
 *
 * @param file a FILE struct opened for the needed file
 *
 * @return the uint32_t read from the file
 */
uint32_t fnReadUint32FromFile(FILE *file) {
	uint8_t bytes[4];
	uint32_t value = 0;

	fread(&bytes, 4, 1, file);

	value |= bytes[0] << 24;
	value |= bytes[1] << 16;
	value |= bytes[2] << 8;
	value |= bytes[3];

	return value;
}

/**
 * Read the first 4 bytes of a file into an uint32_t.
 *
 * @param path the path of the file
 * @param name the name of the file
 *
 * @return the uint32_t read from the file
 */
uint32_t fnReadUint32(const char *path, const char *name) {
	char file_path[MAX_PATH_SIZE];
	FILE *file;

	snprintf(file_path, MAX_PATH_SIZE, "%s/%s", path, name);

	file = fopen(file_path, "rb");
	if (file < 0) {
		return 0;
	}

	uint32_t value = fnReadUint32FromFile(file);

	fclose(file);

	return value;
}

#endif // LINUX_APP
