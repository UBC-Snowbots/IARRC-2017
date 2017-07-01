/*
 * Created by: Robyn Castro
 * Created On: July 1, 2017
 * Description: Takes in an image applies inverse perspective mapping to it
 *
 */

#include <IPMFilter.h>

void IPMFilter::IPMFilter(float ipm_base_width, float ipm_top_width,
                          float ipm_base_displacement, float ipm_top_displacement,
                          float image_height, float image_width) {
    x1 = image_width/2 - ipm_base_width/2 * image_width;
    y1 = (1 - ipm_base_displacement) * image_height;
    x2 = image_width/2 + ipm_base_width/2 * image_width;
    y2 = (1 - ipm_base_displacement) * image_height;
    x3 = image_width/2 + ipm_top_width/2 * image_width;
    y3 = image_height * ipm_top_displacement;
    x4 = image_width/2 - ipm_top_width/2 * image_width;
    y4 = image_height * ipm_top_displacement;
}