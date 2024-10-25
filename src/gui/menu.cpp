#include "menu.h"

#include "stdio.h"
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <math.h>

#ifndef min
#define min(a, b) (a < b ? a : b)
#endif // !min

#ifndef max
#define max(a, b) (a > b ? a : b)
#endif // !max


int menu_x(int menu_x, int menu_alignment, int menu_width, int scale) {

	int x = menu_x * scale;

	if (menu_alignment == ALIGNMENT_LEFT) x += 0;
	else if (menu_alignment == ALIGNMENT_RIGHT) x += menu_width;
	else if (menu_alignment == ALIGNMENT_MIDDLE) x += menu_width / 2;

	return x;
}

int menu_y(int menu_y, int menu_alignment, int menu_height, int scale) {

	int y = menu_y * scale;

	if (menu_alignment == ALIGNMENT_TOP) y += 0;
	else if (menu_alignment == ALIGNMENT_BOTTOM) y += menu_height;
	else if (menu_alignment == ALIGNMENT_MIDDLE) y += menu_height / 2;

	return y;
}


float point_line_distance(int x0, int y0, int x1, int y1, int px, int py) {
    float dx = (float)(x1 - x0);
    float dy = (float)(y1 - y0);
    float num = fabsf(dy * px - dx * py + x1 * y0 - y1 * x0);
    float denom = sqrtf(dx * dx + dy * dy);
    return num / denom;
}

// The function to draw the line with thickness
int draw_line(unsigned int* screen_pixels, int screen_width, int screen_height, 
              int x0, int y0, int x1, int y1, float thickness, unsigned int color) {

    // Determine the bounding box for the line
    int min_x = fmin(x0, x1) - (int)thickness;
    int max_x = fmax(x0, x1) + (int)thickness;
    int min_y = fmin(y0, y1) - (int)thickness;
    int max_y = fmax(y0, y1) + (int)thickness;

    // Clamp the bounding box to the screen dimensions
    min_x = fmax(min_x, 0);
    max_x = fmin(max_x, screen_width - 1);
    min_y = fmax(min_y, 0);
    max_y = fmin(max_y, screen_height - 1);

    // Loop through the bounding box to fill the line
    float half_thickness = thickness / 2.0f;
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            // Check if this pixel is within the line's thickness
            if (point_line_distance(x0, y0, x1, y1, x, y) <= half_thickness) {
                screen_pixels[y * screen_width + x] = color; // Set the pixel to the line color
            }
        }
    }

    return 0; // Success
}



struct menu_item_wrapper {
	int* item;
	int index;
};

int compare(const void* a, const void* b) {
	return (((struct menu_item_wrapper*)a)->item[1] - ((struct menu_item_wrapper*)b)->item[1]);
}

void menu_scene_draw_line(struct menu_line* line, unsigned int* screen, int width, int height, int scale) {

	draw_line(
		screen, 
		width, 
		height, 
		menu_x(line->x0, line->alignment_x, width, scale), 
		menu_y(line->y0, line->alignment_y, height, scale),
		menu_x(line->x1, line->alignment_x, width, scale), 
		menu_y(line->y1, line->alignment_y, height, scale),
		line->thickness, 
		line->color
	);

}

void menu_scene_draw_arrow(struct menu_line* line, unsigned int* screen, int width, int height, int scale) {

	float angle;

	if(line->x1 - line->x0 == 0) angle = -1.5707f;
	else angle = atan((float)(line->y1 - line->y0)/(float)(line->x1 - line->x0));

	struct menu_line arrow_head_1 = {
	-cos(angle -0.8) * 6 * line->thickness+line->x1,
	-sin(angle -0.8) * 6 * line->thickness+line->y1,
	line->x1,
	line->y1,
	line->thickness,
	line->color,
	line->alignment_x,
	line->alignment_y,
	};	

	struct menu_line arrow_head_2 = {
	-cos(angle + 0.8) * 6 * line->thickness+line->x1,
	-sin(angle + 0.8) * 6 * line->thickness+line->y1,
	line->x1,
	line->y1,
	line->thickness,
	line->color,
	line->alignment_x,
	line->alignment_y,
	};	

	menu_scene_draw_line(line, screen, width, height, scale);
	menu_scene_draw_line(&arrow_head_1, screen, width, height, scale);
	menu_scene_draw_line(&arrow_head_2, screen, width, height, scale);


}

void menu_scene_draw_label(struct menu_label* label, unsigned int* screen, int width, int height, int scale, const void** resource_map) {

	pixel_char_print_string(
		label->text, 
		label->text_size * scale, 
		2, 
		menu_x(label->x, label->alignment_x, width, scale), 
		menu_y(label->y, label->alignment_y, height, scale), 
		label->text_alignment_x, 
		label->text_alignment_y, 
		label->max_width,
		label->max_rows,
		screen, 
		width, 
		height, 
		resource_map
	);
}
