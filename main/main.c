#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "lcd.h"
#include "esp_log.h"
#include "esp_timer.h"

// Display config for the SH1106 OLED

#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   1000000
#define SH1106_ADDR          0x3C
#define SH1106_WIDTH         128
#define SH1106_HEIGHT        64
#define SH1106_PAGES         8

// Ray casting config (3D ENGINE)
#define map_W       32
#define map_H       32
#define FOV         (M_PI / 3.0f) // 60 degrees
#define NUM_RAYS    SH1106_WIDTH // 1 ray per column
#define HALF_FOV    (FOV / 2.0f) 
#define MAX_DEPTH   16.0f

// 4x4 Button Matrix pins
#define ROW1        26
#define ROW2        25
#define ROW3        33
#define ROW4        32
#define COL1        19
#define COL2        18
#define COL3        5
#define COL4        17

// Movement config
#define MOVE_SPEED  0.1f
#define TURN_SPEED  0.08f

// Enemies
#define MAX_ENEMIES 8
#define ENEMY_SPEED 0.05f

// FPS counter
static int frame_counter;
static int64_t time_stamp;
static int last_fps;

// z-buffer
static float zbuffer[SH1106_WIDTH];

// Map (1 = wall, 0 = empty)
static const int map[map_H][map_W] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1},
    {1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};

// Player structure
typedef struct{
    float x;
    float y;
    float angle; 
} Player;

static Player player = {
    .x = 3.5f,
    .y = 3.5f,
    .angle = 0.0f,
};

// Player stats (HUD values)
struct player_stats {
    int health;
    int armor;
    int ammo;
};

//Enemy structure + storage
struct enemy {
    float x, y;
    int health;
    int active; // 1 = alive, 0 = dead/inactive
    int damage_tick;
};

struct enemy enemies[MAX_ENEMIES];

// Initialize enemies (all inactive)
void enemies_init(void) {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        enemies[i].health = 50;
        enemies[i].active = 0;
    }
}

// Enemy AI: Move toward player + deal damage
void enemies_update(struct player_stats *stats) {

    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (enemies[i].active == 1) {

            float dx = player.x - enemies[i].x;
            float dy = player.y - enemies[i].y;

            float len = sqrtf(dx * dx + dy * dy);

            if (len > 0.0001f) {
                float inv = 1.0f / len;
                dx *= inv;
                dy *= inv;
            }
            
            // Attempt movement
            float nx = enemies[i].x + dx * ENEMY_SPEED;
            float ny = enemies[i].y + dy * ENEMY_SPEED;
            
            // Collision with map
            if ((int)nx >= 0 && (int)nx < map_W && (int)ny >= 0 && (int)ny < map_H) {
                 if (map[(int)enemies[i].y][(int)nx] != 1) 
                enemies[i].x = nx;

                if (map[(int)ny][(int)enemies[i].x] != 1) 
                    enemies[i].y = ny;
            
            }

            // Damage tick (prevents instant rapid damage)
            enemies[i].damage_tick++;

            float dist_sq = dx * dx + dy * dy;
            float len = sqrtf(dist_sq);

            if (dist_sq < (0.5f * 0.5f)) {
                // deal damage
            }

            // If close to player, deal damage to them
            if (dist_sq < (0.5f * 0.5f) && enemies[i].damage_tick >= 10) {
                stats -> health -= 10;

                if (stats -> health < 0) {
                    stats -> health = 0;
                    enemies[i].damage_tick = 0;
                }
            }
        }

    }

}

// Frame buffer (OLED)
static uint8_t buffer[SH1106_WIDTH * SH1106_PAGES];

// Forward Declarations
static void draw_vline_shaded(uint8_t *buf, int x, int y0, int y1, float dist);
static int show_map = 0;
static int prev_map_btn = 0;
static void render_automap(void);

// I2C / Display drivers 

// Send command to OLED display
static void sh1106_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_write_to_device(I2C_MASTER_NUM, SH1106_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

// Initialize I2C peripherals
static void i2c_init(void) {
    i2c_config_t conf = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = I2C_MASTER_SDA_IO,
        .scl_io_num         = I2C_MASTER_SCL_IO,
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Initialize OLED controller
static void sh1106_init(void) {
    sh1106_cmd(0xAE); // Display off
    sh1106_cmd(0xD5); sh1106_cmd(0x80);
    sh1106_cmd(0xA8); sh1106_cmd(0x3F);
    sh1106_cmd(0xD3); sh1106_cmd(0x00);
    sh1106_cmd(0x40);
    sh1106_cmd(0xAD); sh1106_cmd(0x8B);
    sh1106_cmd(0xA1);
    sh1106_cmd(0xC8);
    sh1106_cmd(0xDA); sh1106_cmd(0x12);
    sh1106_cmd(0x81); sh1106_cmd(0xFF);
    sh1106_cmd(0xD9); sh1106_cmd(0x1F);
    sh1106_cmd(0xDB); sh1106_cmd(0x40);
    sh1106_cmd(0xA4);
    sh1106_cmd(0xA6);
    sh1106_cmd(0xAF); // Display on
}

// Push Frame buffer onto the OLED screen
static void sh1106_draw(uint8_t *buf) {
    for (int page = 0; page < SH1106_PAGES; page++) {
        sh1106_cmd(0xB0 + page);
        sh1106_cmd(0x02);
        sh1106_cmd(0x10);
        uint8_t tmp[SH1106_WIDTH + 1];
        tmp[0] = 0x40;
        memcpy(tmp + 1, buf + (page * SH1106_WIDTH), SH1106_WIDTH);
        i2c_master_write_to_device(I2C_MASTER_NUM, SH1106_ADDR, tmp, sizeof(tmp), pdMS_TO_TICKS(100));
    }
}

// Set for clear pixel in buffer
static void set_pixel(uint8_t *buf, int x, int y, int on) {
    if (x < 0 || x >= SH1106_WIDTH || y < 0 || y >= SH1106_HEIGHT) return;
    if (on)
        buf[(y / 8) * SH1106_WIDTH + x] |= (1 << (y % 8));
    else
        buf[(y / 8) * SH1106_WIDTH + x] &= ~(1 << (y % 8));
}

// RENDERING (3D WALLS)

// Draw vertical wall slice with distance shading
static void draw_vline_shaded(uint8_t *buf, int x, int y0, int y1, float dist) {
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; } // Swap if needed
    for (int y = y0; y <= y1; y++) {
        int draw = 0;

        if (dist < 2.0f) {
            // very close - solid
            draw = 1;
        } else if (dist < 4.0f) {
            //Medium - Checkerboard
            draw = (x + y) % 2 == 0;
        } else if (dist < 6.0f) {
            // Far - Sparse dither
            draw = (x + y) % 4 == 0;
        } else {
            // Very far - barely visible
            draw = (x % 4 == 0) && (y % 4 == 0);
        }
        if (draw) set_pixel(buf, x, y, 1);
    }
}

// Draw the floor(we aint hovering over the void, lol)
static void draw_floor_ceiling(uint8_t *buf, int x, int wall_top, int wall_bottom) {
    // ceiling - Left blank intentionally

    // Floor -dither pattern so it's distinguishable from ceiling
    for (int y = wall_bottom + 1; y < SH1106_HEIGHT; y++) {
        // Checkerboard dither for floor
        if (x % 2 == 0)
            set_pixel(buf, x, y, 1);
    }
}

// INPUT HANDLING (KEYPAD)

// Button Matrix
static void keypad_init(void) {
    // Rows as outputs
    int rows[] = {ROW1, ROW2, ROW3, ROW4};
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(rows[i]);
        gpio_set_direction(rows[i], GPIO_MODE_OUTPUT);
        gpio_set_level(rows[i], 1);
    }
    // Cols as inputs with pull-up
    int cols[] = {COL1, COL2, COL3, COL4};
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(cols[i]);
        gpio_set_direction(cols[i], GPIO_MODE_INPUT);
        gpio_set_level(cols[i], GPIO_PULLUP_ONLY);
    }
}

// Returns 1 if key at row or col is pressed, 0 otherwise.
static int key_pressed(int row_pin, int col_pin) {
    gpio_set_level(row_pin, 0); // Pull row LOW
    int state = !gpio_get_level(col_pin); // pressed = LOW
    gpio_set_level(row_pin, 1); // restore HIGH
    return state;
}

// RAYCASTING CORE

// Cast a single ray. return wall distance
static float cast_ray(float px, float py, float angle) {
    float ray_cos = cosf(angle);
    float ray_sin = sinf(angle);

    float dist = 0.0f;
    for (dist = 0.0f; dist < MAX_DEPTH; dist += 0.05f) {
        float rx = px + ray_cos * dist;
        float ry = py + ray_sin * dist;

        int mx = (int)rx;
        int my = (int)ry;

        if (mx < 0 || mx >= map_W || my < 0 || my >= map_H) return dist;
        if (map[my][mx] == 1) return dist;
    }
    return MAX_DEPTH;
}

// Render full 3D scene
static void render(void) {
    memset(buffer, 0, sizeof(buffer));

    for (int col = 0; col < NUM_RAYS; col++) {
        // Compute ray angle
        float ray_angle = player.angle - HALF_FOV
                        + ((float)col / NUM_RAYS) * FOV;

        float dist = cast_ray(player.x, player.y, ray_angle);

        // Remove fisheye distortion
        dist *= cosf(ray_angle - player.angle);

        // Store Per Column
        zbuffer[col] = dist;

        // Wall height — closer = taller
        int wall_h = (int)(SH1106_HEIGHT / (dist + 0.0001f));
        if (wall_h > SH1106_HEIGHT) wall_h = SH1106_HEIGHT;

        int y0 = (SH1106_HEIGHT / 2) - (wall_h / 2);
        int y1 = (SH1106_HEIGHT / 2) + (wall_h / 2);

        draw_floor_ceiling(buffer, col, y0, y1);
        draw_vline_shaded(buffer, col, y0, y1, dist);
    }

}

void render_sprites(void) {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        float angle_to_enemy = atan2f(enemies[i].y - player.y, enemies[i].x - player.x);
        float angle_diff = angle_to_enemy - player.angle;

        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        float dx = player.x - enemies[i].x;
        float dy = player.y - enemies[i].y;
        float dist_to_player = sqrtf(dx * dx + dy * dy);
        
        int screen_x = (int)((angle_diff / FOV + 0.5f) * SH1106_WIDTH);
        if (enemies[i].active == 1 && screen_x >= 0 && screen_x < SH1106_WIDTH && dist_to_player < zbuffer[screen_x]) {
            int sprite_height = (int)(SH1106_HEIGHT / dist_to_player);
            int sprite_width  = sprite_height / 2;

            int y0 = (SH1106_HEIGHT / 2) - (sprite_height / 2);
            int y1 = (SH1106_HEIGHT / 2) + (sprite_height / 2);

            for (int x = -sprite_width / 2; x <= sprite_width / 2; x++) {
                int draw_x = screen_x + x;

                if (draw_x < 0 || draw_x >= SH1106_WIDTH) continue;

                if (dist_to_player < zbuffer[draw_x]) {
                    draw_vline_shaded(buffer, draw_x, y0, y1, dist_to_player);
                }
            }
        }
    }
}

static void render_automap(void) {
    memset(buffer, 0, sizeof(buffer));

    // Scale the 32x32 map to fit 128x64
    // Each cell = 4px wide, 2px tall (128/32=4, 64/32=2)
    #define CELL_W 4
    #define CELL_H 2

    // Draw map grid
    for (int my = 0; my < map_H; my++) {
        for (int mx = 0; mx < map_W; mx++) {
            if (map[my][mx] == 1) {
                // Draw filled cell for wall
                for (int py = 0; py < CELL_H; py++)
                    for (int px = 0; px < CELL_W; px++)
                        set_pixel(buffer,
                                  mx * CELL_W + px,
                                  my * CELL_H + py, 1);
                        }
        }
    }

    // Draw player position
    int px = (int)(player.x * CELL_W);
    int py = (int)(player.y * CELL_H);

    // Player dot — 3x3 solid square
    for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
        set_pixel(buffer, px + dx, py + dy, 1);

    // Draw line from player to direction
    for (int i = 1; i <= 4; i++) {
        int lx = px + (int)(cosf(player.angle) * i);
        int ly = py + (int)(sinf(player.angle) * i);
        set_pixel(buffer, lx, ly, 1);
    }
    
    // enemy dot
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (enemies[i].active) {
            int ex = (int)(enemies[i].x * CELL_W);
            int ey = (int)(enemies[i].y * CELL_H);
            set_pixel(buffer, ex, ey, 1);
        }
    }

    // Draw "MAP" label top-right
    // Simple pixel font — M
    set_pixel(buffer, 110, 2, 1); set_pixel(buffer, 110, 3, 1);
    set_pixel(buffer, 110, 4, 1); set_pixel(buffer, 110, 5, 1);
    set_pixel(buffer, 110, 6, 1);
    set_pixel(buffer, 111, 3, 1);
    set_pixel(buffer, 112, 2, 1); set_pixel(buffer, 112, 4, 1);
    set_pixel(buffer, 113, 3, 1);
    set_pixel(buffer, 114, 2, 1); set_pixel(buffer, 114, 3, 1);
    set_pixel(buffer, 114, 4, 1); set_pixel(buffer, 114, 5, 1);
    set_pixel(buffer, 114, 6, 1);
    // A
    set_pixel(buffer, 116, 6, 1); set_pixel(buffer, 116, 5, 1);
    set_pixel(buffer, 116, 4, 1); set_pixel(buffer, 116, 3, 1);
    set_pixel(buffer, 116, 2, 1);
    set_pixel(buffer, 117, 2, 1); set_pixel(buffer, 118, 2, 1);
    set_pixel(buffer, 117, 4, 1); set_pixel(buffer, 118, 4, 1);
    set_pixel(buffer, 119, 3, 1); set_pixel(buffer, 119, 5, 1);
    set_pixel(buffer, 119, 6, 1);
    // P
    set_pixel(buffer, 121, 2, 1); set_pixel(buffer, 121, 3, 1);
    set_pixel(buffer, 121, 4, 1); set_pixel(buffer, 121, 5, 1);
    set_pixel(buffer, 121, 6, 1);
    set_pixel(buffer, 122, 2, 1); set_pixel(buffer, 123, 2, 1);
    set_pixel(buffer, 122, 4, 1); set_pixel(buffer, 123, 4, 1);
    set_pixel(buffer, 124, 3, 1);

    sh1106_draw(buffer);

}


void lcd_update_hud(struct player_stats *stats, int fps) {
    lcd_send_byte(0x80, 0);
    char buf[16];
    sprintf(buf, "HP:%-3d AR:%-3d", stats -> health, stats -> armor);
    lcd_print(buf);

    lcd_send_byte(0xC0, 0);
    sprintf(buf, "AM:%-3d FPS:%-3d", stats -> ammo, fps);
    lcd_print(buf);
}

float cast_shoot_ray(void) {
    float ray_cos = cosf(player.angle);
    float ray_sin = sinf(player.angle);

    float dist = 0.0f;
    for (dist = 0.0f; dist < MAX_DEPTH; dist += 0.05f) {
        float rx = player.x + ray_cos * dist;
        float ry = player.y + ray_sin * dist;

        for (int i = 0; i < MAX_ENEMIES; i++) {
            if (enemies[i].active ==1) {
                float dx = enemies[i].x - rx;
                float dy = enemies[i].y - ry;

                if ((dx * dx + dy * dy) < (0.4f * 0.4f)) {
                    enemies[i].health -= 25;
                

                    if (enemies[i].health <= 0) {
                        enemies[i].active = 0;
                    }
                    
                    return dist; //stop ray
                }
            }
        }

        int mx = (int)rx;
        int my = (int)ry;

        if (mx < 0 || mx >= map_W || my < 0 || my >= map_H) return dist;
        if (map[my][mx] == 1) return dist;
    }
    return -1.0f;
 }


// Movement
static void handle_input(struct player_stats *stats) {
    float nx, ny;

    // Sprint Modifier - 1 held
    int sprinting = key_pressed(ROW1, COL1);
    float speed = sprinting ? MOVE_SPEED * 2.0f : MOVE_SPEED;

    // Forward - 2
    if (key_pressed(ROW1, COL2)) {
        nx = player.x + cosf(player.angle) * speed;
        ny = player.y + sinf(player.angle) * speed;
        if (map[(int)ny][(int)nx] == 0) {
            player.x = nx;
            player.y = ny;
        }
    }

    // Backward - 5
    if (key_pressed(ROW2, COL2)) {
        nx = player.x - cosf(player.angle) * speed;
        ny = player.y - sinf(player.angle) * speed;
        if (map[(int)ny][(int)nx] == 0) {
            player.x = nx;
            player.y = ny;
        }
    }

    // Turn left - 4
    if (key_pressed(ROW2, COL1)) {
        player.angle -= TURN_SPEED;
    }

    // Turn right - 6
    if (key_pressed(ROW2, COL3)) {
        player.angle += TURN_SPEED;
    }

    /* Use/interact - 3 (placeholder for now)
    if (key_pressed((ROW2, COL4)) {
        Iteraction Logic goes here later
    }*/

    // Shoot - A (placeholder for now)
    if (key_pressed(ROW4, COL2)) {
        if (stats -> ammo > 0) {
           stats -> ammo --;
            cast_shoot_ray();
        }
    }

    // Map toggle - D (edge triggered, not held)
    int map_btn = key_pressed(ROW4, COL1);
    if (map_btn && !prev_map_btn)
        show_map = !show_map;
    prev_map_btn = map_btn;
}

// MAIN LOOP


void app_main(void) {
    lcd_init();
    lcd_print("DOOM ESP32");
    i2c_init();
    sh1106_init();
    keypad_init();
    enemies_init();
    enemies[0].x = 9.0f;
    enemies[0].y = 4.0f;
    enemies[0].active = 1;

    struct player_stats stats = {100, 0, 50};

    while (1){
        frame_counter++;
        handle_input(&stats);

        static int enemy_tick = 0;
        enemy_tick++;
        if (enemy_tick >= 3) {
            enemies_update(&stats);
            enemy_tick = 0;
        }

        if (show_map) {
            render_automap();
        } else {
            render();
            render_sprites();
            sh1106_draw(buffer);
            // lcd_update_hud(&stats, last_fps);
        }
        int64_t now = esp_timer_get_time();
        if (now - time_stamp >= 1000000) {
            last_fps = frame_counter;
            frame_counter = 0;
            time_stamp = now;
            lcd_update_hud(&stats, last_fps); 
        }
        vTaskDelay(pdMS_TO_TICKS(16));
    }
}

/*int rows[] = {ROW1, ROW2, ROW3, ROW4};
        int cols[] = {COL1, COL2, COL3, COL4};
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                if (key_pressed(rows[r], cols[c])) {
                    printf("ROW%d COL%d\n", r+1, c+1);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));*/