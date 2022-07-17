/**
 * @file lv_port_disp_templ.c
 *
 */

 /*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include "../../lvgl.h"

/*********************
 *      DEFINES
 *********************/
static const char *TAG = "lv_port_disp";

static lv_disp_drv_t disp_drv;
static esp_lcd_panel_handle_t panel_handle = NULL;
static void *p_user_data = NULL;
static bool (*p_on_trans_done_cb)(void *) = NULL;
static SemaphoreHandle_t bsp_lcd_flush_done_sem = NULL;

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static esp_err_t bsp_lcd_set_cb(bool (*trans_done_cb)(void *), void *data);
static bool lcd_trans_done_cb(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *user_data, void *event_data);
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
static bool lv_port_flush_ready(void);
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//        const lv_area_t * fill_area, lv_color_t color);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lv_port_disp_init(void)
{
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << GPIO_LCD_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    ESP_ERROR_CHECK(gpio_set_level(GPIO_LCD_BL, LCD_BL_OFF_LEVEL));

    spi_bus_config_t bus_cfg = {
        .sclk_io_num = GPIO_LCD_SCK,
        .mosi_io_num = GPIO_LCD_SDA,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t)
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = GPIO_LCD_DC,
        .cs_gpio_num = GPIO_LCD_CS,
        .pclk_hz = LCD_SPI_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = lcd_trans_done_cb,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_SPI_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    /**
     * @brief Configure LCD rotation and mirror
     */
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // Turn on backlight (Different LCD screens may need different levels)
    ESP_ERROR_CHECK(gpio_set_level(GPIO_LCD_BL, LCD_BL_ON_LEVEL));

    /**
     * @brief Create mutex to receive LCD flush event.
     */
    if (NULL != bsp_lcd_flush_done_sem) {
        ESP_LOGE(TAG, "LCD already initialized");
    }
    bsp_lcd_flush_done_sem = xSemaphoreCreateBinary();
    if (NULL == bsp_lcd_flush_done_sem) {
        ESP_LOGE(TAG, "No Memory");
    }
    /* If any function is checking LCD trans status before transmition */
    xSemaphoreGive(bsp_lcd_flush_done_sem);

    /* initialize LVGL draw buffers */
    static lv_disp_draw_buf_t draw_buf; 
    lv_color_t *buf1 = heap_caps_malloc(LCD_WIDTH * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);   /* MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT */
    assert(buf1);
#if 1
    lv_color_t *buf2 = heap_caps_malloc(LCD_WIDTH * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_WIDTH * 40);
#else                       
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LCD_WIDTH * 40);   /*Initialize the display buffer*/
#endif

    /* Register the display in LVGL */
    lv_disp_drv_init(&disp_drv);    /*Basic initialization*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;
    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf;

    /* Use lcd_trans_done_cb to inform the graphics library that flush already done */
    bsp_lcd_set_cb(lv_port_flush_ready, NULL);

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static esp_err_t bsp_lcd_flush(int x1, int y1, int x2, int y2, const void *p_data, TickType_t ticks_to_wait)
{
    /* Wait for previous tansmition done */
    if (pdPASS != xSemaphoreTake(bsp_lcd_flush_done_sem, ticks_to_wait)) {
        return ESP_ERR_TIMEOUT;
    }

    return esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2, y2, p_data);
}
// static esp_err_t bsp_lcd_flush_wait_done(TickType_t ticks_to_wait)
// {
//     if (pdPASS != xSemaphoreTake(bsp_lcd_flush_done_sem, ticks_to_wait)) {
//         return ESP_ERR_TIMEOUT;
//     }

//     xSemaphoreGive(bsp_lcd_flush_done_sem);

//     return ESP_OK;
// }

static esp_err_t bsp_lcd_set_cb(bool (*trans_done_cb)(void *), void *data)
{
    if (esp_ptr_executable(trans_done_cb)) {
        p_on_trans_done_cb = trans_done_cb;
        p_user_data = data;
    } else {
        ESP_LOGE(TAG, "Invalid function pointer");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static bool lcd_trans_done_cb(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *user_data, void *event_data)
{
    (void) panel_io;
    (void) user_data;
    (void) event_data;

    /* Used for `bsp_lcd_flush_wait` */
    if (likely(NULL != bsp_lcd_flush_done_sem)) {
        xSemaphoreGive(bsp_lcd_flush_done_sem);
    }

    /* Call user registered function */
    if (NULL != p_on_trans_done_cb) {
        return p_on_trans_done_cb(p_user_data);
    }

    return false;
}

/**
 * @brief Tell LVGL that LCD flush done.
 * 
 * @return true Call `portYIELD_FROM_ISR()` after esp-lcd ISR return.
 * @return false Do nothing after esp-lcd ISR return.v
 */
static bool lv_port_flush_ready(void)
{
    /* Inform the graphics library that you are ready with the flushing */
    lv_disp_flush_ready(&disp_drv);

    /* portYIELD_FROM_ISR (true) or not (false). */
    return false;
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    (void) disp_drv;

    /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/
    bsp_lcd_flush(area->x1, area->y1, area->x2 + 1, area->y2 + 1, (uint8_t *) color_p, portMAX_DELAY);
}

/*OPTIONAL: GPU INTERFACE*/

/*If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color*/
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//                    const lv_area_t * fill_area, lv_color_t color)
//{
//    /*It's an example code which should be done by your GPU*/
//    int32_t x, y;
//    dest_buf += dest_width * fill_area->y1; /*Go to the first line*/
//
//    for(y = fill_area->y1; y <= fill_area->y2; y++) {
//        for(x = fill_area->x1; x <= fill_area->x2; x++) {
//            dest_buf[x] = color;
//        }
//        dest_buf+=dest_width;    /*Go to the next line*/
//    }
//}

/**
 * @brief Task to generate ticks for LVGL.
 * 
 * @param pvParam Not used. 
 */
static void lv_tick_inc_cb(void *data)
{
    uint32_t tick_inc_period_ms = *((uint32_t *) data);

    lv_tick_inc(tick_inc_period_ms);
}

/**
 * @brief Create tick task for LVGL.
 * 
 * @return esp_err_t 
 */
esp_err_t lv_port_tick_init(void)
{
    static const uint32_t tick_inc_period_ms = 5;
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = lv_tick_inc_cb,
            .arg = &tick_inc_period_ms,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "",     /* name is optional, but may help identify the timer when debugging */
            .skip_unhandled_events = true,
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet. Start the timer now */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, tick_inc_period_ms * 1000));

    return ESP_OK;
}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
