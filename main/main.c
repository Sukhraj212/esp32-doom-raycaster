#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "lcd.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

// Display config for the SH1106 OLED

#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000
#define SH1106_ADDR          0x3C
#define SH1106_WIDTH         128
#define SH1106_HEIGHT        64
#define SH1106_PAGES         8

// Ray casting config (3D ENGINE)
#define map_W       32
#define map_H       32
#define FOV         (M_PI / 3.0f) // 60 degrees
#define NUM_RAYS    SH1106_WIDTH
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

// Animations

// Fist
#define FIST_W 16
#define FIST_H 24

#define FIST_RANGE 1.5f

// Enemies animations

// Imp Idle
#define TROO_W 16
#define TROO_H 24

// Imp's projectile
#define FIREBALL_W 8
#define FIREBALL_H 8
#define FIREBALL_FRAME_COUNT 4

// FPS
#define TARGET_FPS      30
#define FRAME_TIME_US   (1000000 / TARGET_FPS)

// Game
#define GAME_SPEED 0.7f

// Enemy's distance from player
#define STOP_DIST   2.2f   // where enemy stops moving
#define ATTACK_DIST 2.4f   // where enemy can deal damage

// FPS counter
static int frame_counter;
static int64_t time_stamp;
static int last_fps;

// Speed
static float deltaTime = 1.0f / TARGET_FPS;
static int prev_shoot_btn = 0;


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

// Helper function for safe map collision checks
static int is_wall(float x, float y) {

    int mx = (int)x;
    int my = (int)y;

    // Treat outside map as solid wall
    if (mx < 0 || mx >= map_W ||
        my < 0 || my >= map_H) {
        return 1;
    }

    return map[my][mx];
}

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

typedef enum {
    ENEMY_IMP,
    ENEMY_TROOPER
} EnemyType;

typedef enum {
    ESTATE_IDLE,
    ESTATE_CHASE,
    ESTATE_ATTACK,
    ESTATE_PAIN,
    ESTATE_DEAD
} EnemyState;


struct enemy {
    float x, y;

    int health;
    int active;

    int damage_tick;

    int anim_frame;
    int anim_timer;

    EnemyType type;

    float speed;
    int damage;

    EnemyState state;
    int state_timer;

    int attack_cooldown;
};

struct enemy enemies[MAX_ENEMIES];

// Weapon states
typedef enum {
    WSTATE_IDLE,
    WSTATE_ATTACK
} WeaponState;

// Weapon frames
typedef enum {
    WFIST_A,
    WFIST_B,
    WFIST_C,
    WFIST_D
} FistFrame;

// Weapon structure
typedef struct {
    WeaponState state;

    int frame;
    int anim_timer;

    int damage_done;
} Weapon;

Weapon fist = {
    .state = WSTATE_IDLE,
    .frame = WFIST_A,
    .anim_timer = 0,
    .damage_done = 0
};

// Projectile structure
typedef enum {
    PTYPE_IMP_FIREBALL
} ProjectileType;

typedef struct projectile {

    bool active;

    ProjectileType type;

    float x;
    float y;

    float dx;
    float dy;

    float speed;

    int damage;

    int anim_frame;
    int anim_timer;
};

#define MAX_PROJECTILES 16

struct projectile projectiles[MAX_PROJECTILES];

void projectiles_init(void) {

    for (int i = 0; i < MAX_PROJECTILES; i++) {

        projectiles[i].active = false;
        projectiles[i].anim_frame = 0;
        projectiles[i].anim_timer = 0;
    }
}

void spawn_projectile(
    float x,
    float y,
    float dx,
    float dy,
    float speed,
    int damage)
{
    for (int i = 0; i < MAX_PROJECTILES; i++) {

        if (!projectiles[i].active) {

            projectiles[i].active = true;

            projectiles[i].type = PTYPE_IMP_FIREBALL;

            projectiles[i].x = x;
            projectiles[i].y = y;

            projectiles[i].dx = dx;
            projectiles[i].dy = dy;

            projectiles[i].speed = speed;
            projectiles[i].damage = damage;

            projectiles[i].anim_frame = 0;
            projectiles[i].anim_timer = 0;

            return;
        }
    }
}

// Initialize enemies (all inactive)
void enemies_init(void) {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        enemies[i].health = 50;
        enemies[i].active = 0;
        enemies[i].anim_frame = 0;
        enemies[i].anim_timer = 0;
        enemies[i].damage_tick = 0;
    }
}

void spawn_enemy(int slot, EnemyType type, float x, float y) {

    enemies[slot].x = x;
    enemies[slot].y = y;

    enemies[slot].active = 1;

    enemies[slot].anim_frame = 0;
    enemies[slot].anim_timer = 0;

    enemies[slot].damage_tick = 0;

    enemies[slot].type = type;

    enemies[slot].state = ESTATE_CHASE;
    enemies[slot].state_timer = 0;

    switch(type) {

        case ENEMY_IMP:
            enemies[slot].health = 60;
            enemies[slot].speed = 0.04f;
            enemies[slot].damage = 10;
            enemies[slot].attack_cooldown = 0;
            break;

        case ENEMY_TROOPER:
            enemies[slot].health = 30;
            enemies[slot].speed = 0.05f;
            enemies[slot].damage = 5;
            break;

        /* case ENEMY_SERGEANT:
            enemies[slot].health = 50;
            enemies[slot].speed = 0.04f;
            enemies[slot].damage = 15;
            break;

        case ENEMY_PINKY:
            enemies[slot].health = 120;
            enemies[slot].speed = 0.06f;
            enemies[slot].damage = 20;
            break;

        case ENEMY_BARON:
            enemies[slot].health = 400;
            enemies[slot].speed = 0.03f;
            enemies[slot].damage = 25;
            break; */
    }
}

// Enemy AI: Move toward player + deal damage
void enemies_update(struct player_stats *stats) {

    for (int i = 0; i < MAX_ENEMIES; i++) {

        if (!enemies[i].active)
            continue;

        // DEAD STATE
        if (enemies[i].state == ESTATE_DEAD) {

            enemies[i].state_timer--;

            if (enemies[i].state_timer <= 0) {
                enemies[i].active = 0;
            }

            continue;
        }

        if (enemies[i].attack_cooldown > 0) {
            enemies[i].attack_cooldown--;
        }

        // PAIN STATE
        if (enemies[i].state == ESTATE_PAIN) {

            enemies[i].state_timer--;

            if (enemies[i].state_timer <= 0) {
                enemies[i].state = ESTATE_CHASE;
            }

            continue;
        }

        // ===== NORMAL CHASE LOGIC BELOW =====

        float dxp = player.x - enemies[i].x;
        float dyp = player.y - enemies[i].y;
        float dist_sq = dxp * dxp + dyp * dyp;

        float dx = dxp;
        float dy = dyp;

        if (dist_sq > 0.0001f) {
            float inv = 1.0f / sqrtf(dist_sq);
            dx *= inv;
            dy *= inv;
        }

        // Animation
        enemies[i].anim_timer++;

        if (enemies[i].anim_timer >= 106) {

            enemies[i].anim_timer = 0;

            switch (enemies[i].state) {

                case ESTATE_CHASE:
                    enemies[i].anim_frame =
                        (enemies[i].anim_frame + 1) % 4;
                    break;

                case ESTATE_ATTACK:
                    if (enemies[i].anim_frame == 0)
                        enemies[i].anim_frame = 1;
                    break;

                case ESTATE_PAIN:
                    enemies[i].anim_frame = 0;
                    break;

                case ESTATE_DEAD:
                    enemies[i].anim_frame = 0;
                    break;

                default:
                    break;
            }
        }

        // ATTACK STATE
        if (enemies[i].state == ESTATE_ATTACK) {

            enemies[i].state_timer--;

            //------------------------------------
            // IMP ATTACK
            //------------------------------------
            if (enemies[i].type == ENEMY_IMP) {

                if (enemies[i].state_timer == 6 &&
                    enemies[i].damage_tick == 0)
                {
                    float dx =
                        player.x - enemies[i].x;

                    float dy =
                        player.y - enemies[i].y;

                    float len =
                        sqrtf(dx*dx + dy*dy);

                    dx /= len;
                    dy /= len;

                    spawn_projectile(
                        enemies[i].x,
                        enemies[i].y,
                        dx,
                        dy,
                        0.08f,
                        8
                    );

                    enemies[i].damage_tick = 15;
                }
            }

            //------------------------------------
            // ALL OTHER ENEMIES
            //------------------------------------
            else {

                if (enemies[i].state_timer == 6 &&
                    enemies[i].damage_tick == 0) {

                    stats->health -= enemies[i].damage;

                    if (stats->health < 0)
                        stats->health = 0;

                    enemies[i].damage_tick = 15;
                }
            }

            if (enemies[i].state_timer <= 0) {

                enemies[i].state = ESTATE_CHASE;
            }

            continue;
        }

        // Movement
        if (dist_sq > STOP_DIST * STOP_DIST) {

            float move_speed =
                enemies[i].speed;

            float nx = enemies[i].x + dx * move_speed;
            float ny = enemies[i].y + dy * move_speed;

            if (!is_wall(nx, ny)) {

                enemies[i].x = nx;
                enemies[i].y = ny;

            } else {

                if (!is_wall(nx, enemies[i].y))
                    enemies[i].x = nx;

                if (!is_wall(enemies[i].x, ny))
                    enemies[i].y = ny;
            }
        }

        // Attack cooldown
        if (enemies[i].damage_tick > 0)
            enemies[i].damage_tick--;

        // Attack
        if (dist_sq < ATTACK_DIST * ATTACK_DIST) {

            if (enemies[i].state != ESTATE_ATTACK && enemies[i].attack_cooldown <= 0) {

                enemies[i].state = ESTATE_ATTACK;

                if (enemies[i].type == ENEMY_IMP) {

                    enemies[i].state_timer = 18;
                    enemies[i].attack_cooldown = 40;

                } else {

                    enemies[i].state_timer = 12;
                    enemies[i].attack_cooldown = 25;
                }

                enemies[i].anim_frame = 0;
                enemies[i].anim_timer = 0;
            }
        }
    }
}


void projectiles_update(struct player_stats *stats)
{
    for (int i = 0; i < MAX_PROJECTILES; i++) {

        if (!projectiles[i].active)
            continue;

        projectiles[i].x +=
            projectiles[i].dx * projectiles[i].speed;

        projectiles[i].y +=
            projectiles[i].dy * projectiles[i].speed;
 
        projectiles[i].anim_timer++;

        if (projectiles[i].anim_timer >= 4) {

            projectiles[i].anim_timer = 0;

            projectiles[i].anim_frame =
                (projectiles[i].anim_frame + 1) % 2;
        }

        // wall collision
        if (is_wall(projectiles[i].x,
                    projectiles[i].y)) {

            projectiles[i].active = false;
            continue;
        }

        // player collision
        float dx = player.x - projectiles[i].x;
        float dy = player.y - projectiles[i].y;

        if (dx*dx + dy*dy < 0.25f) {

            stats->health -= projectiles[i].damage;

            if (stats->health < 0)
                stats->health = 0;

            projectiles[i].active = false;
        }
    }
}


// Frame buffer (OLED)
static uint8_t buffer[SH1106_WIDTH * SH1106_PAGES];

const uint8_t PUNGA0[]  = {
    // image2cpp output 
    0x01, 0x00, 0x05, 0x80, 0x05, 0x80, 0x07, 0xc0, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x03, 0xf0, 
	0x03, 0xf0, 0x03, 0xf0, 0x03, 0xf0, 0x01, 0xf8, 0x01, 0xf8, 0x01, 0xf8, 0x01, 0xfc, 0x01, 0xfc, 
	0x00, 0xfc, 0x00, 0xfe, 0x40, 0xfe, 0xe0, 0xfe, 0xa0, 0xfe, 0xd0, 0xff, 0xf0, 0xff, 0xf8, 0xff
};

const uint8_t PUNGB0[]   = {
    // second frame
    0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0xc0, 0x01, 0xc0, 0x01, 0xe0, 0x03, 0xe0, 0x03, 0xf0, 
	0x03, 0xf8, 0x03, 0xfc, 0x0b, 0xfc, 0x0b, 0xfc, 0x0f, 0xfe, 0x0b, 0xfe, 0x1f, 0xfe, 0x1b, 0xfe, 
	0x1c, 0xff, 0x1f, 0xff, 0x39, 0xff, 0x3b, 0xfe, 0x7f, 0xff, 0x73, 0xdf, 0xff, 0xf6, 0xe7, 0xfe
};

const uint8_t PUNGC0[]   = {
    // third frame
    0x00, 0x3c, 0x00, 0x3c, 0x00, 0x7c, 0x00, 0x7c, 0x00, 0xfc, 0x00, 0xfc, 0x01, 0xfc, 0x01, 0xfe, 
	0x03, 0xfe, 0x03, 0xfe, 0x03, 0xfe, 0x07, 0xfe, 0x07, 0xfe, 0x0f, 0xfe, 0x0f, 0xff, 0x0f, 0xff, 
	0x1f, 0xff, 0x1f, 0xff, 0x3f, 0xfe, 0x3f, 0xfc, 0x7f, 0xf8, 0x7f, 0xf0, 0x7f, 0xa0, 0xff, 0x00
};

const uint8_t PUNGD0[]   = {
    // fourth frame
    0x00, 0x00, 0x00, 0x1e, 0x00, 0x3e, 0x00, 0x77, 0x00, 0xfd, 0x00, 0xff, 0x01, 0xff, 0x01, 0xff, 
	0x03, 0xff, 0x03, 0xfe, 0x07, 0xfe, 0x0f, 0xfe, 0x0f, 0xfe, 0x1f, 0xc2, 0x1f, 0xc0, 0x1f, 0xc0, 
	0x3f, 0x80, 0x3f, 0x80, 0x3f, 0x80, 0x3f, 0x80, 0x7f, 0x80, 0x7f, 0x80, 0x7f, 0x80, 0xff, 0x00

};

static const uint8_t* FIST_FRAMES[] = {
    PUNGA0,
    PUNGB0,
    PUNGC0,
    PUNGD0
};

// 'TROOA1', 16x24px
const uint8_t TROOA1[]   = {
	0x01, 0x80, 0x03, 0xc0, 0x03, 0xe0, 0x03, 0xc0, 0x06, 0xf4, 0x1f, 0xfc, 0x1f, 0x7c, 0x07, 0xf4, 
	0x22, 0x74, 0x03, 0xe4, 0x40, 0x00, 0x41, 0xc1, 0x43, 0xe3, 0x47, 0xb2, 0x47, 0x70, 0x07, 0x30, 
	0x07, 0x20, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x20, 0x01, 0x00, 0x02, 0x80
};
// 'TROOB1', 16x24px
const uint8_t TROOB1[]  = {
	0x03, 0x80, 0x03, 0xc0, 0x03, 0xc0, 0x07, 0xc0, 0x3c, 0xf8, 0x3c, 0xf8, 0x1f, 0xfc, 0x42, 0x74, 
	0x02, 0x46, 0x02, 0x22, 0x41, 0x86, 0x07, 0xae, 0x07, 0xe0, 0x04, 0xf0, 0x04, 0x70, 0x04, 0x70, 
	0x00, 0x70, 0x00, 0x60, 0x05, 0x60, 0x00, 0x70, 0x00, 0x60, 0x00, 0x20, 0x02, 0x20, 0x00, 0x70
};
// 'TROOC1', 16x24px
const uint8_t TROOC1[]  = {
	0x01, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x03, 0xc0, 0x07, 0x34, 0x1f, 0xbc, 0x0e, 0x7c, 0x26, 0x70, 
	0x00, 0x02, 0x41, 0x40, 0xe1, 0x03, 0x67, 0xe2, 0x27, 0x72, 0x07, 0x70, 0x07, 0x70, 0x07, 0x70, 
	0x07, 0x50, 0x01, 0x20, 0x02, 0x20, 0x00, 0xf0, 0x00, 0xa0, 0x01, 0xa0, 0x00, 0x20, 0x00, 0x70
};
// 'TROOD1', 16x24px
const uint8_t TROOD1[]  = {
	0x03, 0x80, 0x03, 0xc0, 0x07, 0xc0, 0x03, 0xc0, 0x1f, 0x3c, 0x1f, 0x3c, 0x1e, 0x78, 0x66, 0x62, 
	0x46, 0x60, 0x64, 0x40, 0x61, 0xc1, 0x00, 0xa0, 0x07, 0xe0, 0x07, 0x00, 0x07, 0x00, 0x07, 0x00, 
	0x07, 0x00, 0x06, 0x00, 0x02, 0x20, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x80
};

// Imp
static const uint8_t* IMP_MOVE[] = {
    TROOA1,
    TROOB1,
    TROOC1,
    TROOD1
};

// 'TROOE1', 16x24px
const uint8_t TROOE1[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x40, 0x00, 0x08, 
	0x01, 0x38, 0x02, 0x80, 0x04, 0x00, 0x08, 0x48, 0x00, 0x30, 0x00, 0x20, 0x00, 0x20, 0x00, 0x40, 
	0x00, 0x44, 0x00, 0x68, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'TROOF1', 16x24px
const uint8_t TROOF1[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 
	0x0b, 0xc0, 0x0f, 0xe0, 0x12, 0xa0, 0x31, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x04, 0x00, 
	0x03, 0x00, 0x03, 0x80, 0x01, 0xa0, 0x01, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'TROOG1', 16x24px
const uint8_t TROOG1[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0x00, 
	0x0c, 0xe0, 0x0e, 0xa0, 0x0f, 0x80, 0x0f, 0x80, 0x06, 0x00, 0x04, 0x02, 0x04, 0x00, 0x08, 0x00, 
	0x30, 0x00, 0x10, 0x00, 0x30, 0x00, 0x10, 0x00, 0x11, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t* IMP_ATTACK[] = {
    TROOE1,
    TROOF1,
    TROOG1
};

// 'TROOH1', 16x24px
const uint8_t TROOH1[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 
	0x03, 0xc8, 0x07, 0xc0, 0x07, 0xdc, 0x39, 0xf0, 0x21, 0xe0, 0x01, 0xc8, 0x00, 0xc8, 0x01, 0xa0, 
	0x03, 0xf0, 0x03, 0xf0, 0x01, 0xe0, 0x01, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t* IMP_PAIN[] = {
    TROOH1
};

// 'TROOI0', 16x24px
const uint8_t TROOI0[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x48, 0x01, 0xc0, 
	0x0d, 0x80, 0x1f, 0xf0, 0x3b, 0xf8, 0x33, 0x00, 0x62, 0x34, 0x03, 0xf0, 0x03, 0xc0, 0x03, 0xc0, 
	0x01, 0x40, 0x07, 0x70, 0x03, 0xf0, 0x01, 0xe0, 0x01, 0xe0, 0x01, 0x40, 0x00, 0x00, 0x00, 0x10
};
// 'TROOJ0', 16x24px
const uint8_t TROOJ0[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xa8, 
	0x03, 0xf8, 0x03, 0x00, 0x06, 0xd0, 0x2f, 0xf8, 0x31, 0xf0, 0x21, 0xc8, 0x01, 0xe8, 0x00, 0xe8, 
	0x07, 0x20, 0x03, 0xf0, 0x01, 0xf0, 0x01, 0xe0, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'TROOK0', 16x24px
const uint8_t TROOK0[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x3c, 0x00, 0x9c, 0x08, 0xf8, 0x00, 0xd0, 0x03, 0xc0, 0x02, 0x00, 0x0c, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x10, 0x0e, 0x10, 0x1c, 0x10, 0x08, 0x30, 0x00, 0xf0, 0x00, 0x00, 0x00
};
// 'TROOL0', 16x24px
const uint8_t TROOL0[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 
	0x00, 0x70, 0x00, 0x40, 0x01, 0xd8, 0x03, 0xf0, 0x03, 0xb0, 0x00, 0xe0, 0x03, 0xe0, 0x06, 0x03, 
	0x88, 0x07, 0x70, 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'TROOM0', 16x24px
const uint8_t TROOM0[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x60, 0x01, 0x70, 0x00, 0x98, 0xc3, 0xec, 0xce, 0x6c, 0x60, 0x0c, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


static const uint8_t* IMP_DEATH[] = {
    TROOI0,
    TROOJ0,
    TROOK0,
    TROOL0,
    TROOM0
};

// Imp projectile
// 'BAL1A0', 8x8px
const uint8_t  BAL1A0[]  = {
	0x18, 0x7e, 0x46, 0xdb, 0xdb, 0x66, 0x3e, 0x18
};
// 'BAL1B0', 8x8px
const uint8_t  BAL1B0[]  = {
	0x18, 0x7e, 0x6a, 0xdf, 0xdf, 0x7a, 0x7e, 0x18
};
// 'BAL1C0', 8x8px
const uint8_t  BAL1C0[]  = {
	0x10, 0x3c, 0x7c, 0x76, 0x7e, 0x3c, 0x00, 0x00
};
// 'BAL1D0', 8x8px
const uint8_t  BAL1D0[]  = {
	0x10, 0x3c, 0x7e, 0x42, 0x66, 0x6e, 0x18, 0x00
};
// 'BAL1E0', 8x8px
const uint8_t BAL1E0[]  = {
	0x00, 0x20, 0x02, 0x00, 0x40, 0x00, 0x00, 0x00
};

const uint8_t*  allArray[] = {
	 BAL1A0,
	 BAL1B0,
	 BAL1C0,
	 BAL1D0,
	 BAL1E0
};

// Zombie man

// 'POSSA1', 16x24px
const uint8_t POSSA1[]  = {
	0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x4f, 0x10, 0x2f, 0x00, 
	0x1e, 0x18, 0xa5, 0x00, 0x99, 0x80, 0x33, 0x80, 0x3b, 0xc0, 0x37, 0xc0, 0x17, 0x80, 0x1d, 0x80, 
	0x0f, 0x80, 0x0f, 0x80, 0x03, 0x80, 0x07, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'POSSB1', 16x24px
const uint8_t POSSB1[]  = {
	0x00, 0x00, 0x05, 0x00, 0x03, 0x80, 0x00, 0x00, 0x02, 0x00, 0x03, 0x00, 0x63, 0xb0, 0x2d, 0xac, 
	0x3c, 0x30, 0x84, 0x20, 0x01, 0xe0, 0x77, 0xc0, 0x23, 0xe0, 0x1b, 0xf0, 0x0b, 0xe0, 0x0f, 0x80, 
	0x0f, 0x20, 0x0e, 0xe0, 0x06, 0xe0, 0x03, 0xe0, 0x02, 0x00, 0x02, 0x40, 0x00, 0x40, 0x00, 0x40
};
// 'POSSC1', 16x24px
const uint8_t POSSC1[]  = {
	0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x55, 0x00, 0x1f, 0x90, 0x39, 0x00, 
	0x00, 0x68, 0x00, 0x10, 0x03, 0xc0, 0x37, 0xc0, 0x3f, 0xc0, 0x34, 0x00, 0x06, 0x40, 0x1f, 0x00, 
	0x1f, 0x80, 0x1f, 0x80, 0x0f, 0x80, 0x0d, 0x00, 0x0d, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x00, 0x00
};
// 'POSSD1', 16x24px
const uint8_t POSSD1[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x05, 0x00, 0x7d, 0x60, 
	0x19, 0x90, 0x9e, 0x00, 0x80, 0x80, 0x3b, 0x80, 0x33, 0x80, 0x1e, 0xc0, 0x3f, 0x80, 0x1f, 0x00, 
	0x3e, 0x80, 0x1f, 0x80, 0x1f, 0x00, 0x0f, 0x00, 0x11, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t* ZOMBIE_MOVE[] = {
    POSSA1,
    POSSB1,
    POSSC1,
    POSSD1
};

// 'POSSE1', 16x24px
const uint8_t POSSE1[]  = {
	0x00, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0c, 0x00, 0x09, 0x08, 0x0c, 0xb0, 
	0x00, 0x50, 0x28, 0x10, 0x1e, 0x78, 0x09, 0xc0, 0x0f, 0x48, 0x06, 0x58, 0x07, 0x40, 0x06, 0x40, 
	0x01, 0x58, 0x07, 0xb0, 0x07, 0xf8, 0x01, 0xf8, 0x00, 0xf8, 0x00, 0xd0, 0x00, 0x00, 0x00, 0x00
};
// 'POSSF1', 16x24px
const uint8_t POSSF1[]  = {
	0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x06, 0x04, 0x00, 0x7c, 0x04, 0x90, 
	0x04, 0xd8, 0x00, 0x38, 0x1a, 0x78, 0x00, 0xc0, 0x06, 0x84, 0x02, 0x7c, 0x07, 0x4c, 0x02, 0x04, 
	0x00, 0x38, 0x03, 0xf8, 0x07, 0xf8, 0x01, 0xf8, 0x01, 0xf8, 0x00, 0xe8, 0x00, 0x10, 0x00, 0x08
};

static const uint8_t* ZOMBIE_ATTACK[] = {
    POSSE1,
    POSSF1
};

// 'POSSG1', 16x24px
const uint8_t POSSG1[]  = {
	0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x08, 0x00, 0x30, 0x00, 0xe0, 0x03, 0xe0, 0x00, 0x6e, 
	0x00, 0x0b, 0x60, 0x07, 0x43, 0x1c, 0x0b, 0xcc, 0x02, 0xec, 0x03, 0xdc, 0x02, 0xfc, 0x00, 0xa4, 
	0x03, 0xbc, 0x03, 0x98, 0x03, 0xd8, 0x01, 0x9c, 0x01, 0x80, 0x01, 0x80, 0x00, 0x00, 0x00, 0x18
};

static const uint8_t* ZOMBIE_PAIN[] = {
    POSSG1
};

// 'POSSH0', 16x24px
const uint8_t POSSH0[]  = {
	0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x10, 0x00, 0x20, 0x00, 0xe0, 0x01, 0xe0, 0x02, 0x4c, 
	0x07, 0x1a, 0x0a, 0x4c, 0x09, 0x22, 0x03, 0x04, 0x00, 0x1c, 0x10, 0xfc, 0x05, 0xc8, 0x2e, 0x88, 
	0x06, 0xb8, 0x03, 0xb8, 0x03, 0xb8, 0x01, 0xb8, 0x01, 0x90, 0x01, 0x80, 0x00, 0x10, 0x00, 0x10
};
// 'POSSI0', 16x24px
const uint8_t POSSI0[]  = {
	0x00, 0x00, 0x00, 0x10, 0x00, 0x40, 0x00, 0x40, 0x00, 0xe0, 0x06, 0xc5, 0x0e, 0xd1, 0x0b, 0x70, 
	0x1f, 0x72, 0x1f, 0x3c, 0x3f, 0x70, 0x3c, 0xd8, 0x3e, 0x80, 0x79, 0xf0, 0x70, 0xc0, 0x70, 0xe0, 
	0x70, 0xe0, 0x70, 0x40, 0x20, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'POSSJ0', 16x24px
const uint8_t POSSJ0[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x07, 0x4a, 0x06, 0xf0, 0x0f, 0xfc, 0x1c, 0x80, 0x3d, 0xf0, 0x38, 0xe0, 0x70, 0xe0, 0x60, 0xe0, 
	0x00, 0x40, 0xc0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'POSSK0', 16x24px
const uint8_t POSSK0[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x02, 0x94, 0x0f, 0xf8, 0x0f, 0xf8, 0x1d, 0x70, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'POSSL0', 16x24px
const uint8_t POSSL0[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xa0, 0x07, 0xb4, 0x3e, 0xf8, 0x3e, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t* ZOMBIE_DEATH[] = {
    POSSH0,
    POSSI0,
    POSSJ0,
    POSSK0,
    POSSL0
};


// Forward Declarations
static void draw_vline_shaded(uint8_t *buf, int x, int y0, int y1, float dist);
static int show_map = 0;
static int prev_map_btn = 0;
static void render_automap(void);

float cast_shoot_ray(void);

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

static void draw_scaled_sprite(
    uint8_t *buf,
    int center_x,
    int center_y,
    int target_w,
    int target_h,
    const uint8_t *sprite,
    int src_w,
    int src_h,
    int color
) {

    int start_x = center_x - (target_w / 2);
    int start_y = center_y - (target_h / 2);

    for (int y = 0; y < target_h; y++) {
        for (int x = 0; x < target_w; x++) {

            int src_x = (x * src_w) / target_w;
            int src_y = (y * src_h) / target_h;

            int byte_index =
                src_y * (src_w / 8) + (src_x / 8);

            int bit = 7 - (src_x % 8);

            if (sprite[byte_index] & (1 << bit)) {
                set_pixel(buf, start_x + x, start_y + y, color);
            }
        }
    }
}

static void render_weapon(void) {

    const uint8_t *sprite = FIST_FRAMES[fist.frame];

    int offset_x = 0;
    int offset_y = 0;

    int fist_size = 40;

    // Weapon bob
    static int bob = 0;
    bob++;

    int bob_y = (int)(sinf(bob * 0.15f) * 2);

    switch (fist.frame) {

        case WFIST_B:
            offset_x = -10;
            offset_y = 2;
            break;

        case WFIST_C:
            offset_x = -38;
            offset_y = -2;
            break;

        case WFIST_D:
            offset_x = -30;
            offset_y = 1;
            break;

        default:
            break;
    }

    // Bigger impact frame
    if (fist.frame == WFIST_C) {
        fist_size = 48;
    }

    // Shadow / outline
    draw_scaled_sprite(
        buffer,

        (SH1106_WIDTH / 2) + 20 + offset_x + 1,
        SH1106_HEIGHT - 10 + offset_y + bob_y + 1,

        fist_size,
        fist_size,

        sprite,
        FIST_W,
        FIST_H,
        0
    );

    // Main sprite
    draw_scaled_sprite(
        buffer,

        (SH1106_WIDTH / 2) + 20 + offset_x,
        SH1106_HEIGHT - 10 + offset_y + bob_y,

        fist_size,
        fist_size,

        sprite,
        FIST_W,
        FIST_H,
        2
    );
}


void update_fist(void) {

    if (fist.state == WSTATE_IDLE)
        return;

    fist.anim_timer++;

    if (fist.anim_timer < 7)
        return;

    fist.anim_timer = 0;

    switch (fist.frame) {

        case WFIST_B:
            fist.frame = WFIST_C;
            break;

        case WFIST_C:

            // Apply damage exactly once
            if (!fist.damage_done) {
                cast_shoot_ray();
                fist.damage_done = 1;
            }

            fist.frame = WFIST_D;
            break;

        case WFIST_D:

            // End attack
            fist.state = WSTATE_IDLE;
            fist.frame = WFIST_A;
            break;

        case WFIST_A:
        default:
            break;
    }
}

// Shadow outline for enemeies
static void draw_sprite_shadow(
    uint8_t *buf,
    int x,
    int y,
    int w,
    int h,
    const uint8_t *sprite,
    int src_w,
    int src_h
) {
    // shadow pass (offset +1, +1)
    for (int sy = 0; sy < h; sy++) {
        for (int sx = 0; sx < w; sx++) {

            int src_x = (sx * src_w) / w;
            int src_y = (sy * src_h) / h;

            int byte_index = src_y * (src_w / 8) + (src_x / 8);
            int bit = 7 - (src_x % 8);

            if (sprite[byte_index] & (1 << bit)) {
                set_pixel(buf, x + sx + 1, y + sy + 1, 0);
            }
        }
    }

    // main pass
    for (int sy = 0; sy < h; sy++) {
        for (int sx = 0; sx < w; sx++) {

            int src_x = (sx * src_w) / w;
            int src_y = (sy * src_h) / h;

            int byte_index = src_y * (src_w / 8) + (src_x / 8);
            int bit = 7 - (src_x % 8);

            if (sprite[byte_index] & (1 << bit)) {
                set_pixel(buf, x + sx, y + sy, 1);
            }
        }
    }
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
        gpio_set_pull_mode(cols[i], GPIO_PULLUP_ONLY);
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
static float cast_ray(float px, float py, float dirX, float dirY) {

    int mapX = (int)px;
    int mapY = (int)py;

    float invDirX = (dirX == 0.0f) ? 1e30f : 1.0f / dirX;
    float invDirY = (dirY == 0.0f) ? 1e30f : 1.0f / dirY;   

    float deltaDistX = fabsf(invDirX);
    float deltaDistY = fabsf(invDirY);

    int stepX, stepY;
    float sideDistX, sideDistY;

    if (dirX < 0) {
        stepX = -1;
        sideDistX = (px - mapX) * deltaDistX;
    } else {
        stepX = 1;
        sideDistX = (mapX + 1.0f - px) * deltaDistX;
    }

    if (dirY < 0) {
        stepY = -1;
        sideDistY = (py - mapY) * deltaDistY;
    } else {
        stepY = 1;
        sideDistY = (mapY + 1.0f - py) * deltaDistY;
    }

    int hit = 0;
    int side = 0;

    while (!hit) {
        if (sideDistX < sideDistY) {
            sideDistX += deltaDistX;
            mapX += stepX;
            side = 0;
        } else {
            sideDistY += deltaDistY;
            mapY += stepY;
            side = 1;
        }

        if (mapX < 0 || mapX >= map_W || mapY < 0 || mapY >= map_H)
            return MAX_DEPTH;

        if (map[mapY][mapX] == 1)
            hit = 1;
    }

    if (side == 0)
        return (mapX - px + (1 - stepX) / 2) / dirX;
    else
        return (mapY - py + (1 - stepY) / 2) / dirY;
}

// Render full 3D scene
static void render(void) {
    memset(buffer, 0, sizeof(buffer));

    float start_angle = player.angle - HALF_FOV;
    float step = FOV / NUM_RAYS;

    float cos_step = cosf(step);
    float sin_step = sinf(step);

    // initial ray direction
    float dirX = cosf(start_angle);
    float dirY = sinf(start_angle);

    // for fisheye correction (also incremental)
    float cos_player = cosf(player.angle);
    float sin_player = sinf(player.angle);

    for (int col = 0; col < NUM_RAYS; col++) {

        float dist = cast_ray(player.x, player.y, dirX, dirY);

        // fisheye correction using dot product (NO trig)
        float dot = dirX * cos_player + dirY * sin_player;
        dist *= dot;

        zbuffer[col] = dist;

        int wall_h = (int)(SH1106_HEIGHT / (dist + 0.0001f));
        if (wall_h > SH1106_HEIGHT) wall_h = SH1106_HEIGHT;

        int y0 = (SH1106_HEIGHT / 2) - (wall_h / 2);
        int y1 = (SH1106_HEIGHT / 2) + (wall_h / 2);

        draw_floor_ceiling(buffer, col, y0, y1);
        draw_vline_shaded(buffer, col, y0, y1, dist);

        // rotate ray direction for next column
        float newX = dirX * cos_step - dirY * sin_step;
        float newY = dirX * sin_step + dirY * cos_step;
        dirX = newX;
        dirY = newY;
    }
}

void render_sprites(void) {
    typedef struct {
        int index;
        float dist;
    } EnemySort;

    EnemySort sorted[MAX_ENEMIES];
    int count = 0;

    // Collect active enemies
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (enemies[i].active) {
            float dx = player.x - enemies[i].x;
            float dy = player.y - enemies[i].y;
            sorted[count].index = i;
            sorted[count].dist = dx * dx + dy * dy; // no sqrt needed
            count++;
        }
    }

    // Sort far → near
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (sorted[j].dist < sorted[j + 1].dist) {
                EnemySort tmp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = tmp;
            }
        }
    }

    // Render in sorted order
    for (int k = 0; k < count; k++) {
        int i = sorted[k].index;

        float angle_to_enemy = atan2f(enemies[i].y - player.y,
                                      enemies[i].x - player.x);
        float angle_diff = angle_to_enemy - player.angle;

        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        float dx = player.x - enemies[i].x;
        float dy = player.y - enemies[i].y;
        float dist_to_player = sqrtf(sorted[k].dist);

        int screen_x = (int)((angle_diff / FOV + 0.5f) * SH1106_WIDTH);
        
        int col = screen_x;

        if (col < 0 || col >= SH1106_WIDTH) continue;

        if (dist_to_player >= zbuffer[col]) continue;

        int sprite_height = (int)(SH1106_HEIGHT / dist_to_player);

        if (sprite_height > 48)
            sprite_height = 48;
            
        int sprite_width = (TROO_W * sprite_height) / TROO_H;



        if (screen_x < -sprite_width || screen_x >= SH1106_WIDTH + sprite_width) continue;

        int y0 = (SH1106_HEIGHT / 2) - (sprite_height / 2);

        const uint8_t *sprite = NULL;

        switch(enemies[i].type) {

            case ENEMY_IMP:

                switch(enemies[i].state)
                {
                    case ESTATE_CHASE:
                        sprite = IMP_MOVE[enemies[i].anim_frame % 4];
                        break;

                    case ESTATE_ATTACK:
                        sprite = IMP_ATTACK[enemies[i].anim_frame % 3];
                        break;

                    case ESTATE_PAIN:
                        sprite = IMP_PAIN[0];
                        break;

                    case ESTATE_DEAD:
                        sprite = IMP_DEATH[enemies[i].anim_frame % 5];
                        break;

                    default:
                        sprite = IMP_MOVE[0];
                        break;
                } 

                break;

            case ENEMY_TROOPER:

                switch(enemies[i].state)
                {
                    case ESTATE_CHASE:
                        sprite = ZOMBIE_MOVE[enemies[i].anim_frame % 4];
                        break;

                    case ESTATE_ATTACK:
                        sprite = ZOMBIE_ATTACK[enemies[i].anim_frame % 2];
                        break;

                    case ESTATE_PAIN:
                        sprite = ZOMBIE_PAIN[0];
                        break;

                    case ESTATE_DEAD:
                        sprite = ZOMBIE_DEATH[enemies[i].anim_frame % 5];
                        break;

                    default:
                        sprite = ZOMBIE_MOVE[0];
                        break;
                }

                break;

            /* case ENEMY_SERGEANT:
                sprite = TROO_FRAMES[enemies[i].anim_frame % 4];
                break;

            case ENEMY_PINKY:
                sprite = TROO_FRAMES[enemies[i].anim_frame % 4];
                break;

            case ENEMY_BARON:
                sprite = TROO_FRAMES[enemies[i].anim_frame % 4];
                break; */
        }
        

        for (int x = -sprite_width / 2; x <= sprite_width / 2; x++) {
            draw_sprite_shadow(
                buffer,
                screen_x - sprite_width / 2,
                y0,
                sprite_width,
                sprite_height,
                sprite,
                TROO_W,
                TROO_H
            );
            
            /*int draw_x = screen_x + x;
            if (draw_x < 0 || draw_x >= SH1106_WIDTH) continue;
            if (dist_to_player >= zbuffer[draw_x]) continue;

            for (int y = 0; y < sprite_height; y++) {
                int draw_y = y0 + y;
                if (draw_y < 0 || draw_y >= SH1106_HEIGHT) continue;

                int tex_x = (x + sprite_width / 2) * TROO_W / sprite_width;
                int tex_y = y * TROO_H / sprite_height;

                int bytes_per_row = (TROO_W + 7) / 8;
                int byte_index = tex_y * bytes_per_row + (tex_x / 8);

                int bit = 7 - (tex_x % 8);

                if (sprite[byte_index] & (1 << bit)) {
                    set_pixel(buffer, draw_x, draw_y, 1);
                }
            }*/
        } 
    }
}

void render_projectiles(void)
{
    for (int i = 0; i < MAX_PROJECTILES; i++) {

        if (!projectiles[i].active)
            continue;

        float angle_to_proj =
            atan2f(projectiles[i].y - player.y,
                   projectiles[i].x - player.x);

        float angle_diff = angle_to_proj - player.angle;

        while (angle_diff > M_PI)
            angle_diff -= 2.0f * M_PI;

        while (angle_diff < -M_PI)
            angle_diff += 2.0f * M_PI;

        if (fabsf(angle_diff) > HALF_FOV)
            continue;

        
        float dx = player.x - projectiles[i].x;
        float dy = player.y - projectiles[i].y;

        float dist_to_player = dx * dx + dy * dy;

        if (dist_to_player < 0.1f)
        dist_to_player = 0.1f;

        int screen_x =
            (int)((angle_diff / FOV + 0.5f) * SH1106_WIDTH);

        //----------------------------------------
        // sprite size
        //----------------------------------------

        int sprite_height = (int)(20.0f / dist_to_player);

        if (sprite_height < 20)
            sprite_height = 20;

        int sprite_width = (FIREBALL_W * sprite_height) / FIREBALL_H;

        int y0 = (SH1106_HEIGHT / 2) - (sprite_height / 2);

        //----------------------------------------
        // current animation frame
        //----------------------------------------

        const uint8_t *sprite =
            allArray[projectiles[i].anim_frame % 5];

        //----------------------------------------
        // draw columns
        //----------------------------------------

        for (int x = -sprite_width / 2;
             x <= sprite_width / 2;
             x++) {

            int draw_x = screen_x + x;

            if (draw_x < 0 || draw_x >= SH1106_WIDTH)
                continue;

            // hidden behind wall?
            if (dist_to_player >= zbuffer[draw_x])
                continue;

            for (int y = 0; y < sprite_height; y++) {

                int draw_y = y0 + y;

                if (draw_y < 0 || draw_y >= SH1106_HEIGHT)
                    continue;

                int tex_x =
                    (x + sprite_width / 2)
                    * FIREBALL_W / sprite_width;

                int tex_y =
                    y * FIREBALL_H / sprite_height;

                int bytes_per_row =
                    (FIREBALL_W + 7) / 8;

                int byte_index =
                    tex_y * bytes_per_row
                    + (tex_x / 8);

                int bit =
                    7 - (tex_x % 8);

                if (sprite[byte_index] & (1 << bit)) {

                    set_pixel(
                        buffer,
                        draw_x,
                        draw_y,
                        1
                    );
                }
            }
        }
    }
}

void update_projectiles(struct player_stats *stats)
{
    for(int i=0;i<MAX_PROJECTILES;i++)
    {
        if(!projectiles[i].active)
            continue;

        //--------------------------------
        // movement
        //--------------------------------

        projectiles[i].x +=
            projectiles[i].dx * projectiles[i].speed;

        projectiles[i].y +=
            projectiles[i].dy * projectiles[i].speed;

        //--------------------------------
        // hit wall
        //--------------------------------

        if(is_wall(projectiles[i].x,
                   projectiles[i].y))
        {
            projectiles[i].active = false;
            continue;
        }

        //--------------------------------
        // hit player
        //--------------------------------

        float dx =
            player.x-projectiles[i].x;

        float dy =
            player.y-projectiles[i].y;

        if(dx*dx+dy*dy < 0.25f)
        {
            stats->health -= projectiles[i].damage;

            projectiles[i].active=false;

            continue;
        }

        //--------------------------------
        // animation
        //--------------------------------

        projectiles[i].anim_timer++;

        if(projectiles[i].anim_timer>=4)
        {
            projectiles[i].anim_timer=0;

            projectiles[i].anim_frame++;

            if(projectiles[i].anim_frame>=FIREBALL_FRAME_COUNT)
                projectiles[i].anim_frame=0;
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
    for (dist = 0.0f; dist < FIST_RANGE; dist += 0.05f) {
        float rx = player.x + ray_cos * dist;
        float ry = player.y + ray_sin * dist;

        int mx = (int)rx;
        int my = (int)ry;

        if (mx < 0 || mx >= map_W || my < 0 || my >= map_H) return dist;
        if (map[my][mx] == 1) return dist;

        for (int i = 0; i < MAX_ENEMIES; i++) {
            if (enemies[i].active ==1) {
                float dx = enemies[i].x - rx;
                float dy = enemies[i].y - ry;

                if ((dx * dx + dy * dy) < (0.4f * 0.4f)) {
                    enemies[i].health -= 25;
                

                    if (enemies[i].health <= 0) {

                        enemies[i].health = 0;

                        enemies[i].state = ESTATE_DEAD;
                        enemies[i].state_timer = 30;

                    } else {

                        enemies[i].state = ESTATE_PAIN;
                        enemies[i].state_timer = 10;
                    }
                    
                    return dist; //stop ray
                }
            }
        }

    }
    return -1.0f;
}

void fist_attack(void) {

    if (fist.state != WSTATE_IDLE)
        return;

    fist.state = WSTATE_ATTACK;

    fist.frame = WFIST_B;

    fist.anim_timer = 0;
    fist.damage_done = 0;
}


// Movement
static void handle_input(struct player_stats *stats) {
    float nx, ny;

    // Sprint Modifier - 1 held
    int sprinting = key_pressed(ROW1, COL1);
    float speed = MOVE_SPEED * (sprinting ? 2.0f : 1.0f);

    // Forward - 2
    if (key_pressed(ROW1, COL2)) {
        nx = player.x + cosf(player.angle) * speed;
        ny = player.y + sinf(player.angle) * speed;

        if (!is_wall(nx, player.y))
            player.x = nx;

        if (!is_wall(player.x, ny))
            player.y = ny;

    }

    // Backward - 5
    if (key_pressed(ROW2, COL2)) {
        nx = player.x - cosf(player.angle) * speed;
        ny = player.y - sinf(player.angle) * speed;

        if (!is_wall(nx, player.y))
            player.x = nx;

        if (!is_wall(player.x, ny))
            player.y = ny;

    }

    // Turn left - 4
    if (key_pressed(ROW2, COL1)) {
        player.angle -= TURN_SPEED;

        if (player.angle < 0)
            player.angle += 2.0f * M_PI;
    }

    // Turn right - 6
    if (key_pressed(ROW2, COL3)) {
        player.angle += TURN_SPEED;

        if (player.angle >= 2.0f * M_PI)
            player.angle -= 2.0f * M_PI;
    }

    /* Use/interact - 3 (placeholder for now)
    if (key_pressed((ROW2, COL4)) {
        Iteraction Logic goes here later
    }*/

    // Shoot - A
    int shoot_btn = key_pressed(ROW1, COL4);

    if (shoot_btn && !prev_shoot_btn) {
        fist_attack();
    }

    prev_shoot_btn = shoot_btn;

    // Map toggle - D (edge triggered, not held)
    int map_btn = key_pressed(ROW4, COL4);
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
    projectiles_init();
    spawn_enemy(0, ENEMY_IMP, 9.0f, 4.0f);
    spawn_enemy(0, ENEMY_TROOPER, 8.0f, 8.0f);
    spawn_enemy(1, ENEMY_IMP, 10.0f, 8.0f);
    spawn_enemy(2, ENEMY_TROOPER, 15.0f, 10.0f);
    spawn_enemy(3, ENEMY_IMP, 20.0f, 15.0f);

    struct player_stats stats = {100, 0, 50};

    while (1) {

        int64_t frame_start = esp_timer_get_time();

        handle_input(&stats);
        update_fist();

        enemies_update(&stats);
        projectiles_update(&stats);
        update_projectiles(&stats);

        if (show_map) {
            render_automap();
        } else {
            render();
            render_sprites();
            render_projectiles();
            render_weapon();
            sh1106_draw(buffer);
        }

        // FPS counter
        frame_counter++;
        int64_t now = esp_timer_get_time();
        if (now - time_stamp >= 1000000) {
            last_fps = frame_counter;
            frame_counter = 0;
            time_stamp = now;
            lcd_update_hud(&stats, last_fps);
        }

        // FRAME LIMITER (correct version)
        int64_t frame_end = esp_timer_get_time();
        int64_t frame_time = frame_end - frame_start;

        if (frame_time < FRAME_TIME_US) {

            int delay_ms =
                (FRAME_TIME_US - frame_time) / 10000;

            if (delay_ms > 0)
                vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
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