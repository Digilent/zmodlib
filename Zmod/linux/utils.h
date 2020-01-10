/**
 * @file utils.h
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 * @date 15 Nov 2019
 * @brief File containing util functions for linux-specific implementations.
 */

#ifdef LINUX_APP

#ifndef UTILS_H
#define UTILS_H

#define MAX_PATH_SIZE 512

bool fnStartsWith(const char *name, const char *prefix);
uint32_t fnReadUint32FromFile(FILE *file);
uint32_t fnReadUint32(const char *path, const char *name);

#endif // UTILS_H

#endif // LINUX_APP
