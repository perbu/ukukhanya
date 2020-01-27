#include <stdio.h>
#include <strings.h>
#include <ctype.h>

#include "mgos.h"
#include "mgos_ro_vars.h"
#include "mgos_app.h"
#include "mgos_system.h"
#include "mgos_timers.h"

#include "mgos_shadow.h"

#include <mgos_neopixel.h>
#include <frozen.h>

#include "mgos_mqtt.h"

#include "hsv2rgb.h"

// Consts for the event type. Perhaps I should have used enums.
#define MQTT_SRC   0
#define SHADOW_SRC 1

int blink_counter; // Counts up and down.
bool blink_up;     // Are we counting up or down?

static struct pixel *pixels_shadow; // persistent pixels
static struct pixel *pixels_mqtt; // streaming pixels

static struct mgos_neopixel *my_strip; // Low-level led strip.

static void turn_off_status_led(void *param)
{
    mgos_gpio_write(mgos_sys_config_get_pins_statusLed(), 0);
}

void blink_status_led()
{
    mgos_gpio_write(mgos_sys_config_get_pins_statusLed(), 1);
    mgos_set_timer(100, 0, turn_off_status_led, NULL);
}

/* 
    Sets a subpixel to a specific value.

    X is used for other stats. For streamed values, it keeps track of decay.
    for shadow state it indicates a blinking state.

*/
void set_pixel_color(struct pixel *pixel_array, bool set_decay, int number, char key, int value)
{

    switch (key)
    {
    case 'r':
        pixel_array[number].r = value;
        break;
    case 'g':
        pixel_array[number].g = value;
        break;
    case 'b':
        pixel_array[number].b = value;
        break;
    case 'x':
        pixel_array[number].x = value;
        break;
    default:
        LOG(LL_ERROR, ("Internal brain damage - unknown color '%c'", key));
    }
    // If this comes via streamed set initial X value to 255
    // This will decay
    if (set_decay)
    {
        pixel_array[number].decay = 255;
    }
}

/* 
    js_token_cb is given to the frozen JSON parser and 
    gets fired off whenever it has a token for us.

*/

static void js_token_cb(void *callback_data,
                        const char *name, size_t name_len,
                        const char *path, const struct json_token *token)
{

    int event_type = *(int*) callback_data;
    // LOG(LL_INFO, ("JS token callback type %d", event_type));
    if (token->type == JSON_TYPE_NUMBER)
    {
        int pixelid = 0;
        char color;
        int value;
        // this might accept some whitespace here and there. Unix.
        int ret = sscanf(path, ".leds.%i.%c", &pixelid, &color);
        //color = tolower(color);
        // Only proceed if we got two values from sscanf and we're talking about a known color.
        // Indicates blinking state.
        if (ret == 2 && (color == 'r' || color == 'g' || color == 'b' || color == 'x'))
        {
            value = atoi(token->ptr);
            // This is where the magix happens:
            if (event_type == MQTT_SRC) {
                LOG(LL_DEBUG, ("Setting streamed LED %d (%c): %d", pixelid, color, value));
                set_pixel_color(pixels_mqtt, true, pixelid, color, value);
            } else {
                LOG(LL_DEBUG, ("Setting shadow state LED %d (%c): %d", pixelid, color, value));
                set_pixel_color(pixels_shadow, false, pixelid, color, value);
            }
        }
    }
}

int still_streaming()
{
    int active_pixels = 0;
    for (int i = 0; i < mgos_sys_config_get_leds_number(); i++)
    {
        if (pixels_mqtt[i].decay)
            active_pixels++;
    }
    return (active_pixels);
}

static void decay_stream()
{
    for (int i = 0; i < mgos_sys_config_get_leds_number(); i++)
    {
        int new_decay = (pixels_mqtt[i].decay * (255 - mgos_sys_config_get_leds_decay()) / 255 );

        pixels_mqtt[i].decay = new_decay;
        // if we're below 10 we might as well slam it down to 0.
        if (pixels_mqtt[i].decay < 10)
        {
            pixels_mqtt[i].decay = 0;
        }
    }
}
/* 
    Update the blink_counter for shadow state blinking leds.
*/

static void blink_shadow()
{
    if (blink_up)
    {
        blink_counter += mgos_sys_config_get_leds_blinkstep();
        if (blink_counter > 255)
        {
            blink_counter = 255;
            blink_up = false;
        }
    }
    else
    {
        if (blink_counter > mgos_sys_config_get_leds_blinkstep())
        {
            blink_counter -= mgos_sys_config_get_leds_blinkstep();
        }
        else
        {
            blink_counter = 0;
            blink_up = true;
        }
    }
}

/*
    syncs up our internal datastructures with the neopixel libs.

*/
static void show(struct pixel *pixel_array, bool stream)
{
    if (stream) // Show the streaming state.
    {
        for (int i = 0; i < mgos_sys_config_get_leds_number(); i++)
        {
            mgos_neopixel_set(my_strip, i,
                              (pixel_array[i].r * pixel_array[i].decay) / 255,
                              (pixel_array[i].g * pixel_array[i].decay) / 255,
                              (pixel_array[i].b * pixel_array[i].decay) / 255);
        }
        mgos_neopixel_show(my_strip); // Show it!
    }
    else
    {
        // Show shadow state.
        for (int i = 0; i < mgos_sys_config_get_leds_number(); i++)
        {

            LOG(LL_VERBOSE_DEBUG, ("Syncing pixel %d (%d:%d:%d)", i,
                                   pixel_array[i].r, pixel_array[i].g, pixel_array[i].b));

            if (pixel_array[i].x)
            {
                // This is a blinking pixel. Render it by taking the blink_factor into account.

                mgos_neopixel_set(my_strip, i,
                                  (pixel_array[i].r * blink_counter) / 255,
                                  (pixel_array[i].g * blink_counter) / 255,
                                  (pixel_array[i].b * blink_counter) / 255);
            }
            else
            {
                mgos_neopixel_set(my_strip, i,
                                  pixel_array[i].r,
                                  pixel_array[i].g,
                                  pixel_array[i].b);
            }
        }
        LOG(LL_VERBOSE_DEBUG, ("Showing new pixels (mgos_neopixel_show)"));
        mgos_neopixel_show(my_strip);
    }
}

static void update_neopixels_cb(void *data)
{
    int streamed_pixels = still_streaming();
    if (streamed_pixels > 0)
    {
        show(pixels_mqtt, true);
        decay_stream();
    }
    else
    {
        show(pixels_shadow, false);
        blink_shadow();
    }
}

/* 
    Make a rainbow on bootup
    Let it fade out

*/
static void mk_rainbow()
{
    float f = 360.0 / (float)mgos_sys_config_get_leds_number();

    for (int i = 0; i < mgos_sys_config_get_leds_number(); i++)
    {
        struct pixel p;
        // Create a rainbox with full brightness and saturation:
        HSVtoRGB((int)(i * f), 1.0, 1.0, &p);
        set_pixel_color(pixels_shadow, true, i, 'r', p.r);
        set_pixel_color(pixels_shadow, true, i, 'g', p.g);
        set_pixel_color(pixels_shadow, true, i, 'b', p.b);
        show(pixels_shadow, false);
        mgos_msleep(20);        
    }
    for (int i = 0; i < mgos_sys_config_get_leds_number(); i++)
    {
        // Create a rainbox with full brightness and saturation:
        set_pixel_color(pixels_shadow, true, i, 'r', 0);
        set_pixel_color(pixels_shadow, true, i, 'g', 0);
        set_pixel_color(pixels_shadow, true, i, 'b', 0);
        show(pixels_shadow, false);
        mgos_msleep(20);        
    }
}

/* 
   hsv_stream_handler - handle a message with RGB color space. 
*/

static void rgb_stream_handler(struct mg_connection *nc, const char *topic, int topic_len, const char *msg, int msg_len, void *ud)
{
    LOG(LL_INFO, ("RGB handler got message topic '%.*s' message: '%.*s'", topic_len, topic, msg_len, msg));
    blink_status_led();
    int event_type = MQTT_SRC;
    int ret = json_walk(msg, msg_len, js_token_cb, &event_type);
    LOG(LL_INFO, ("Scanned MQTT event and found %d entries", ret));
}

static void delta_cb(int ev, void *ev_data, void *userdata)
{
    int event_type = SHADOW_SRC;
    struct mg_str *delta = (struct mg_str *)ev_data;
    blink_status_led();
    LOG(LL_INFO, ("Delta callback: len: %d delta: '%s'", (int)delta->len, delta->p));
    int ret = json_walk(delta->p, (int)delta->len, js_token_cb, &event_type);
    LOG(LL_INFO, ("Scanned shadow delta and found %d entries", ret));

    (void)ev;
    (void)userdata;
}

/* static void sub

    connection, fmt, ..

*/
static void setup_mqtt()
{
    char rgbtopic[] = "my/rgb-stream";
    mgos_mqtt_sub(rgbtopic, rgb_stream_handler, NULL);
    LOG(LL_INFO, ("Subscribed to '%s'", rgbtopic));
}
static void setup_shadow()
{
    mgos_event_add_handler(MGOS_SHADOW_UPDATE_DELTA, delta_cb, NULL);
}

enum mgos_app_init_result mgos_app_init(void)
{
    blink_status_led();
    LOG(LL_INFO, ("My device ID is: %s", mgos_sys_config_get_device_id())); // Get config param
    LOG(LL_INFO, ("Initializing. Using pin %i for status.", mgos_sys_config_get_pins_statusLed()));
    mgos_gpio_set_mode(mgos_sys_config_get_pins_statusLed(), MGOS_GPIO_MODE_OUTPUT);
    LOG(LL_INFO, ("LED strip on PIN %i", mgos_sys_config_get_pins_ledStrip()));

    // Allocate and clear the pixel arrays.
    pixels_mqtt = (struct pixel *)calloc(mgos_sys_config_get_leds_number(), sizeof(struct pixel)); // Allocate pixel array
    pixels_shadow = (struct pixel *)calloc(mgos_sys_config_get_leds_number(), sizeof(struct pixel)); // Allocate pixel array

    // Initialize the mongoose neopixel datastructures.
    my_strip = mgos_neopixel_create(mgos_sys_config_get_pins_ledStrip(),
                                    mgos_sys_config_get_leds_number(), MGOS_NEOPIXEL_ORDER_GRB);

    setup_mqtt();
    setup_shadow();
    LOG(LL_INFO, ("Init hopefully successful"));


    mk_rainbow();

    mgos_set_timer(mgos_sys_config_get_leds_updateinterval(),
                   MGOS_TIMER_REPEAT, update_neopixels_cb, NULL);

    blink_status_led();
    return MGOS_APP_INIT_SUCCESS;
}
