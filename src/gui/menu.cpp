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


int draw_line(unsigned int* screen_pixels, int screen_width, int screen_height, 
              int x0, int y0, int x1, int y1, float thickness, unsigned int color) {
    if (screen_pixels == NULL || screen_width <= 0 || screen_height <= 0) {
        return -1; // invalid parameters
    }

    // Calculate the direction vector and length
    float dx = x1 - x0;
    float dy = y1 - y0;
    float length = sqrtf(dx * dx + dy * dy);
    if (length == 0.0f) return 0; // points are the same; nothing to draw

    // Normalize direction vector
    float ux = dx / length;
    float uy = dy / length;

    // Calculate perpendicular offset for line thickness
    float half_thickness = thickness / 2.0f;
    float offset_x = -uy * half_thickness;
    float offset_y = ux * half_thickness;

    // Define the rectangle corners around the line
    int x0a = (int)(x0 + offset_x);
    int y0a = (int)(y0 + offset_y);
    int x0b = (int)(x0 - offset_x);
    int y0b = (int)(y0 - offset_y);
    int x1a = (int)(x1 + offset_x);
    int y1a = (int)(y1 + offset_y);
    int x1b = (int)(x1 - offset_x);
    int y1b = (int)(y1 - offset_y);

    // Helper function to draw a filled triangle
    auto draw_triangle = [&](int x0, int y0, int x1, int y1, int x2, int y2) {
        // Bounding box for the triangle
        int minX = fmaxf(0, fminf(x0, fminf(x1, x2)));
        int maxX = fminf(screen_width - 1, fmaxf(x0, fmaxf(x1, x2)));
        int minY = fmaxf(0, fminf(y0, fminf(y1, y2)));
        int maxY = fminf(screen_height - 1, fmaxf(y0, fmaxf(y1, y2)));

        for (int y = minY; y <= maxY; y++) {
            for (int x = minX; x <= maxX; x++) {
                // Compute barycentric weights
                float w0 = (x1 - x0) * (y - y0) - (x - x0) * (y1 - y0);
                float w1 = (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1);
                float w2 = (x0 - x2) * (y - y2) - (x - x2) * (y0 - y2);

                // Inside test
                if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                    screen_pixels[y * screen_width + x] = color;
                }
            }
        }
    };

    // Draw two triangles to cover the rectangle
    draw_triangle(x0a, y0a, x0b, y0b, x1a, y1a);
    draw_triangle(x1b, y1b, x1a, y1a, x0b, y0b);

    return 0;
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

	double mag = sqrtf64((double)(line->x1 - line->x0) * (double)(line->x1 - line->x0) + (double)(line->y1 - line->y0) * (double)(line->y1 - line->y0));

	if(mag < 1. && mag > -1.) {
		struct menu_line arrow_head_1 = {
			line->x1 - (double)line->thickness * 3.,
			line->y1 + (double)line->thickness * 3.,
			line->x1,
			line->y1,
			line->thickness,
			line->color,
			line->alignment_x,
			line->alignment_y,
		};	

		struct menu_line arrow_head_2 = {
			line->x1 + (double)line->thickness * 3.,
			line->y1 + (double)line->thickness * 3.,
			line->x1,
			line->y1,
			line->thickness,
			line->color,
			line->alignment_x,
			line->alignment_y,
		};	

		menu_scene_draw_line(&arrow_head_1, screen, width, height, scale);
		menu_scene_draw_line(&arrow_head_2, screen, width, height, scale);
		return;
	}

	struct menu_line arrow_head_1 = {
	line->x1 + (double)(line->x0 - line->x1) / mag * (double)line->thickness * 3. - (double)(line->y0 - line->y1) / mag * (double)line->thickness * 3.,
	line->y1 + (double)(line->x0 - line->x1) / mag * (double)line->thickness * 3. + (double)(line->y0 - line->y1) / mag * (double)line->thickness * 3.,
	line->x1,
	line->y1,
	line->thickness,
	line->color,
	line->alignment_x,
	line->alignment_y,
	};	

	struct menu_line arrow_head_2 = {
	line->x1 + (double)(line->x0 - line->x1) / mag * (double)line->thickness * 3. + (double)(line->y0 - line->y1) / mag * (double)line->thickness * 3.,
	line->y1 - (double)(line->x0 - line->x1) / mag * (double)line->thickness * 3. + (double)(line->y0 - line->y1) / mag * (double)line->thickness * 3.,
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
