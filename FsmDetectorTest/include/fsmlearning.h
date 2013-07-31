#ifndef __FSMLEARNING_H
#define __FSMLEARNING_H

//void test_graphOpt();
//void test_mkStatFromImageList(const char *offImageFilenameList, const char *onImageFilenameList);
void test_learnFromImagesAndMasks(const int firstFileIndex, const int lastFileIndex, const char *configFileName = NULL);

#endif