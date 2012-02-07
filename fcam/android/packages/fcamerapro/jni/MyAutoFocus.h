#ifndef _MYAUTOFOCUS_H
#define _MYAUTOFOCUS_H

#include <FCam/Tegra/AutoFocus.h>
#include <FCam/Tegra/Lens.h>
#include <FCam/Base.h>
#include <android/log.h>

#define NUM_INTERVALS 23
#define RECT_EDGE_LEN 70
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define FILTER_SIZE 5
#define NUM_NIGHT_IMAGES 2

typedef unsigned char uchar;


static const float discreteDioptres[] = {0.2f, 0.25f, 0.33f, 0.5f, 1.0f, 1.11f, 1.25f, 1.42f, 1.66f, 2.0f,
										 2.5f, 3.33f, 5.0f, 5.26f, 5.56f, 5.88f, 6.25f, 6.67f, 7.14f, 7.69f,
										 8.33f, 9.09f, 10.0f};

class MyAutoFocus : public FCam::Tegra::AutoFocus {
public:

       MyAutoFocus(FCam::Tegra::Lens *l, FCam::Rect r = FCam::Rect()) : FCam::Tegra::AutoFocus(l,r) {
    	   lens = l;
    	   rect = r;
    	   /* [CS478]
    	    * Do any initialization you need.
    	    */
    	   night = false;
    	   for (int i = 0; i < NUM_INTERVALS; i++)
    		   sharpVals[i] = 0.0f;
       }

       bool isFocusing()
       {
    	   return focusing;
       }

       void setNightMode(bool isNight)
       {
    	   night = isNight;
       }

       /* Looks through contrast values, and picks the index with the highest contrast value.
        * This index later is mapped to a focal length value
        */
       int findMaxIdx()
       {
    	   int max = -1;
    	   int maxIdx = -1;

    	   //Catch outliers for the first checkpoint
    	   if ((sharpVals[0] * 0.6f) > sharpVals[1])
    		   sharpVals[0] = sharpVals[1];

    	   for (int i = 0; i < NUM_INTERVALS; i++)
    	   {
    		   if (sharpVals[i] > max)
    		   {
    			   max = sharpVals[i];
    			   maxIdx = i;
    		   }
    	   }
    	   logArrayDump();
    	   LOG("MYFOCUS max idx : %d\n", maxIdx);
    	   return maxIdx;
       }

       /* Sets focus region. For global, it's the entire frame */
       void setRect(int x, int y, int width = RECT_EDGE_LEN, int height = RECT_EDGE_LEN)
       {
    	   rect.x = std::max(x - RECT_EDGE_LEN / 2, 0);
    	   rect.y = std::max(y - RECT_EDGE_LEN / 2, 0);
    	   rect.width = std::min(width + rect.x, IMAGE_WIDTH) - rect.x;
    	   rect.height = std::min(height + rect.y, IMAGE_HEIGHT) - rect.y;
    	   //logRectDump();
       }

       /* Clear the stored region in preparation for the next focal length value
        */
       void clearNightRegion()
       {
    	   for(int i = 0; i < RECT_EDGE_LEN; i++)
    	   {
    		   for(int j = 0; j < RECT_EDGE_LEN; j++)
    		   {
    			   nightRegion[i][j] = 0;
    		   }
    	   }
       }

       /* Store image region for night mode (stores multiple frame data) */
       void saveImage(FCam::Image &image){

    	   int highX = rect.x + rect.width;
    	   int highY = rect.y + rect.height;
    	   for(int x = rect.x; x < highX; x++)
    	   {
    		   for(int y = rect.y; y < highY; y++)
    		   {
    			   nightRegion[x-rect.x][y-rect.y] += *(image(x,y));
    		   }
    	   }
    	   logNightRegionDump();
       }

       /* Beging focusing */
       void startSweep() {
    	   if (!lens) return;
    	   /* [CS478]
    	    * This method should initiate your autofocus algorithm.
    	    * Before you do that, do basic checks, e.g. is the autofocus
    	    * already engaged?
    	    */
    	   if (focusing) return;
    	   if (night)
    		   clearNightRegion();
    	   bestFocalDist = -1.0f;

    	   itvlCount = 0;
    	   lens->setFocus(discreteDioptres[itvlCount]);
    	   itvlCount++;
    	   nightCount = 0;

    	   focusing = true;
    	   logDump();
       }

       /* High Freq Pass filter - averages a region of pixels and takes the difference
        * between the center of the pixel and the average
        */
       int computeImageContrast(FCam::Image &image)
       {
    	   LOG("MYFOCUS compute contrast begin\n======================\n");

    	   unsigned int sum = 0;
    	   int totalValue = 0;
    	   int filterHalfSize = FILTER_SIZE / 2;
    	   int filterArea = FILTER_SIZE * FILTER_SIZE;
    	   int highX = rect.x + rect.width - filterHalfSize;
    	   int highY = rect.y + rect.height - filterHalfSize;
    	   for(int x = rect.x + filterHalfSize; x < highX; x++)
    	   {
    		   for(int y = rect.y + filterHalfSize; y < highY; y++)
    		   {
    			   sum = 0;
    			   for (int i = -filterHalfSize; i <= filterHalfSize; i++)
    				   for (int j = -filterHalfSize; j <= filterHalfSize; j++)
    					   sum += *image(x+i, y+j);

    			   sum /= filterArea;
    			   int temp = *image(x, y) - sum;

    			   totalValue += temp * temp;
    		   }
    	   }
    	   LOG("MYFOCUS total value: %d\n", totalValue);
    	   LOG("MYFOCUS compute contrast end\n======================\n");
    	   return totalValue;
       }

       /* Similar to computeImage contrast, but operatores on the stored data for
        * night mode
        */
       int computeNightContrast(FCam::Image &image){

    	   saveImage(image);

    	   LOG("MYFOCUS night contrast begin\n======================\n");

    	   unsigned int sum = 0;
    	   int totalValue = 0;
    	   int filterHalfSize = FILTER_SIZE / 2;
    	   int filterArea = FILTER_SIZE * FILTER_SIZE;
    	   int highX = rect.width - filterHalfSize;
    	   int highY = rect.height - filterHalfSize;
    	   for(int x = filterHalfSize; x < highX; x++)
    	   {
    		   for(int y = filterHalfSize; y < highY; y++)
    		   {
    			   sum = 0;
    			   for (int i = -filterHalfSize; i <= filterHalfSize; i++)
    				   for (int j = -filterHalfSize; j <= filterHalfSize; j++)
    					   sum += nightRegion[x+i][y+j];

    			   sum /= filterArea;
    			   int temp = nightRegion[x][y] - sum;

    			   totalValue += temp * temp;
    		   }
    	   }
    	   LOG("MYFOCUS total value: %d\n", totalValue);
    	   LOG("MYFOCUS night contrast end\n======================\n");
    	   return totalValue;
       }

       void update(const FCam::Frame &f) {
    	   /* [CS478]
    	    * This method is supposed to be called in order to inform
    	    * the autofocus engine of a new viewfinder frame.
    	    * You probably want to compute how sharp it is, and possibly
    	    * use the information to plan where to position the lens next.
    	    */
    	   /* Extract frame and do stuff */

    	   FCam::Image image = f.image();
    	   if (!image.valid()){
    		   sharpVals[itvlCount-1] = -1;
    		   LOG("MYFOCUS Invalid image\n");
    	   }

    	   float expectedFocus = discreteDioptres[itvlCount - 1];
    	   LOG("MYFOCUS The expected focus setting: %f\n", expectedFocus);
    	   float actualFocus = (float) f["lens.focus"];
    	   LOG("MYFOCUS The average focus setting during the frame: %f\n", actualFocus);

    	   //If the lens focus request didn't go through, try again
    	   if ((actualFocus > expectedFocus + 0.003f) || (actualFocus < expectedFocus - 0.003f))
    	   {
    		   LOG("MYFOCUS Trying lens focus request again\n");
    		   lens->setFocus(expectedFocus);
    		   return;
    	   }

    	   logDump();

    	   if (night && nightCount < NUM_NIGHT_IMAGES - 1){
    		   saveImage(image);
    		   nightCount++;
    		   return;
    	   }

    	   if (night && nightCount == NUM_NIGHT_IMAGES - 1){
    		   sharpVals[itvlCount-1] = computeNightContrast(image);
    		   clearNightRegion();
    		   nightCount = 0;
    	   }
    	   else
    		   sharpVals[itvlCount-1] = computeImageContrast(image);

    	   if (itvlCount != NUM_INTERVALS){
    		   lens->setFocus(discreteDioptres[itvlCount]);
    		   itvlCount++;
    		   return;
    	   }
    	   int maxIdx = findMaxIdx();//TODO change this meaning
    	   logDump();

		   bestFocalDist = discreteDioptres[maxIdx];
		   LOG("MYFOCUS The best focus setting: %f\n", bestFocalDist);
		   focusing = false;
		   lens->setFocus(bestFocalDist);
       }

       /************************** Debugging methods ***************************/
       void logNightRegionDump()
       {
    	   LOG("MYFOCUS LOG NIGHT REG DUMP BEGIN\n======================\n");
    	   for(int j = 10; j < 30; j++)
    	   {
    		   LOG("MYFOCUS LOG NIGHT val: %d\n", nightRegion[30][j]);
    	   }
    	   LOG("MYFOCUS LOG NIGHT REG DUMP END\n======================\n");
       }

       void logDump()
       {
    	   LOG("MYFOCUS LOG DUMP BEGIN\n======================\n");
    	   LOG("MYFOCUS interval count: %d\n", itvlCount);
    	   LOG("MYFOCUS Focusing?: %d\n", focusing);
    	   LOG("MYFOCUS night count: %d\n", nightCount);
    	   LOG("MYFOCUS night?: %d\n", night);
    	   LOG("MYFOCUS LOG DUMP END\n======================\n");
       }

       void logArrayDump()
       {
    	   LOG("MYFOCUS LOG ARRAY DUMP BEGIN\n======================\n");
    	   for (int i = 0; i < NUM_INTERVALS; i++)
    	   {
    		   LOG("MYFOCUS array index: %d, val: %d\n", i, sharpVals[i]);
    	   }
    	   LOG("MYFOCUS LOG ARRAY DUMP END\n======================\n");
       }

       void logRectDump()
       {
    	   LOG("MYFOCUS LOG RECT DUMP BEGIN\n======================\n");
    	   LOG("MYFOCUS rect x: %d\n", rect.x);
    	   LOG("MYFOCUS rect y: %d\n", rect.y);
    	   LOG("MYFOCUS rect width: %d\n", rect.width);
    	   LOG("MYFOCUS rect height: %d\n", rect.height);
    	   LOG("MYFOCUS LOG RECT DUMP END\n======================\n");
       }


private:
       FCam::Tegra::Lens* lens;
       FCam::Rect rect;
       /* [CS478]
        * Declare any state variables you might need here.
        */
       int itvlCount;
       int sharpVals[NUM_INTERVALS];
       float nearFocus;
       float farFocus;
       float bestFocalDist;
       bool focusing;
       bool night;
       int nightCount;
       int nightRegion[RECT_EDGE_LEN][RECT_EDGE_LEN];
};

#endif
