#include "driver/pwm.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"
#include "ros_comms.h"
// #include "CytronMotorDriver.h"

// extern uint32_t esp_get_time(void);

static const char* TAG = "ros-comms";

#define GPIO_PWM_R GPIO_NUM_14 
#define GPIO_DIR1_R GPIO_NUM_13 

#define GPIO_PWM_L GPIO_NUM_12 // pwm-pwm
#define GPIO_PWM2_L GPIO_NUM_15

#define GPIO_EN1_L GPIO_NUM_4
#define GPIO_EN2_L GPIO_NUM_5 



// ROSserial elements
ros::NodeHandle nh;

ros::Time last_call_time;
std_msgs::String status_msg;

ros::Publisher status_pub("/roboy/pinky/middleware/espchair/status", &status_msg);

// Init PWM Parameters

uint32_t duties[N_PWM_PINS] = { 0, 0, 0};

float phases[N_PWM_PINS] = { 0, 0, 0};

static const uint32_t pwm_limit_val = PWMLIMIT;

static const int pwm_lim_top = pwm_limit_val;
static const int pwm_lim_bot = -1*(pwm_limit_val);

static bool emergency_stop_active = false;

static int counter = 0;

static int last_left_cmd_counter = 0;
static int last_right_cmd_counter = 0;

esp_timer_handle_t timer_handle;

// GPIO List
const gpio_num_t gpio_pins[6] = {
  GPIO_PWM_R, 
  GPIO_DIR1_R,
  
  GPIO_PWM_L,
  GPIO_PWM2_L,
  GPIO_EN1_L,
  GPIO_EN2_L
    
};

void pwm_update_R( const std_msgs::Int16& drive_R )
{
  last_right_cmd_counter = counter;
  
  int pwm_tmp = drive_R.data;
  int msg_len = 100;
  char message[msg_len];
  snprintf(message, msg_len, "Got motor right value: %d\n", pwm_tmp);
  printf(message);
  status_msg.data = message;
  status_pub.publish(&status_msg);

  if ( pwm_tmp > pwm_lim_top )     // Clamping
  {
    // ESP_LOGW(TAG, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", pwm_tmp, pwm_lim_top);
    pwm_tmp = pwm_lim_top;
    snprintf(message, msg_len, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", pwm_tmp, pwm_lim_top);
    status_msg.data = message;
    status_pub.publish(&status_msg);
  }
  if ( pwm_tmp < pwm_lim_bot )
  {
    // ESP_LOGW(TAG, "Warning, PWM of %d below pwm_lim_bot", pwm_tmp);
    pwm_tmp = pwm_lim_bot;
    snprintf(message, msg_len, "Warning, PWM of %d below pwm_lim_bot", pwm_tmp);
    status_msg.data = message;
    status_pub.publish(&status_msg);
  }

  

  if (pwm_tmp >= 0 )    // Direction reversal
  {
    
    //duties[2] = 0;
    duties[0] = pwm_tmp;
    gpio_set_level(GPIO_DIR1_R,0);

  }else{
    pwm_tmp *= -1;
    // //duties[0] = 0;
    duties[0] = pwm_tmp;
    
    gpio_set_level(GPIO_DIR1_R,1);

  }


  //ESP_ERROR_CHECK( esp_timer_stop(timer_handle) ); //Feed the timer
  //ESP_ERROR_CHECK( esp_timer_start_once(timer_handle, TIMEOUT_IN_US) );

  // ESP_LOGI(TAG, "duties0: %d, duties1: %d, duties2: %d, duties3: %d",duties[0],duties[1],duties[2],duties[3]);
  // ESP_LOGI(TAG, "pwm_tmp = %d, with PWMLIM %d, duties_0 = %d", pwm_tmp, pwm_limit_val, duties[0]);

  if ( !emergency_stop_active )
  {
    ESP_ERROR_CHECK( pwm_set_duties(duties) );
    ESP_ERROR_CHECK( pwm_start() );
  } else {
    duties[0] = 0;
     duties[1] = 0;    
     duties[2]=0;              //Set all PWM to 0
    for ( int i = 0; i < 4; i++){       //Ensure pwm=0
      
      gpio_set_level(gpio_pins[i],0);   //Disable all low side switches
    }
  }

}

void pwm_update_L( const std_msgs::Int16& drive_L )
{
  last_left_cmd_counter = counter;
  int pwm_tmp = drive_L.data*-1;//*0.889; // fix, since the left motor is 91 RPM and right is 80 RPM
  int msg_len = 100;
  char message[msg_len];
  snprintf(message, msg_len, "Got motor left value: %d\n", pwm_tmp);
  //printf(message);
  status_msg.data = message;
  status_pub.publish(&status_msg);
  if ( pwm_tmp > pwm_lim_top )     // Clamping
  {
    // ESP_LOGW(TAG, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", pwm_tmp, pwm_lim_top);
    pwm_tmp = pwm_lim_top;
    snprintf(message, msg_len, "Warning, sent PWM of %d over top limit, pwm_lim_top = %d", pwm_tmp, pwm_lim_top);
    status_msg.data = message;
    status_pub.publish(&status_msg);
  }
  if ( pwm_tmp < pwm_lim_bot )
  {
    // ESP_LOGW(TAG, "Warning, PWM of %d below pwm_lim_bot", pwm_tmp);
    pwm_tmp = pwm_lim_bot;
    snprintf(message, msg_len, "Warning, PWM of %d below pwm_lim_bot", pwm_tmp);
    status_msg.data = message;
    status_pub.publish(&status_msg);
  }


  // PWM-PWM mode
  if (pwm_tmp >= 0 )    // Direction reversal
  {
    duties[1] = pwm_tmp;
    duties[2] = 0;

  }else{              

    pwm_tmp *= -1;
    duties[1] = 0;
    duties[2] = pwm_tmp;

  }

  // ESP_ERROR_CHECK( esp_timer_stop(timer_handle) ); //Feed the timer
  // ESP_ERROR_CHECK( esp_timer_start_once(timer_handle, TIMEOUT_IN_US) );

  // ESP_LOGI(TAG, "duties0: %d, duties1: %d, ",duties[0],duties[1]);
  // ESP_LOGI(TAG, "pwm_tmp = %d, with PWMLIM %d, duties_1 = %d", pwm_tmp, pwm_limit_val, duties[1]);

  if ( !emergency_stop_active )
  {
    gpio_set_level(GPIO_EN1_L,1);
    gpio_set_level(GPIO_EN2_L,1);
    ESP_ERROR_CHECK( pwm_set_duties(duties) );
    ESP_ERROR_CHECK( pwm_start() );
  } else {
     duties[0] = 0;
     duties[1] = 0;      
    for ( int i = 0; i < 4; i++){       //Ensure pwm=0
      
      gpio_set_level(gpio_pins[i],0);   //Disable all low side switches
    }
  }
}

// void e_stop( const std_msgs::Empty& e_stop_flag )
// {
//   int msg_len = 100;
//   char message[msg_len];
//   gpio_set_level(GPIO_NUM_15,0);      //Disconnect main relay

//   emergency_stop_active = true;
//   duties[0] = 0;
//   duties[1] = 0; 
//   for ( int i = 0; i < 4; i++){    
//     gpio_set_level(gpio_pins[i],1);   //Disable all low side switches
//   }

//   ESP_ERROR_CHECK( pwm_set_duties(duties) );
//   ESP_ERROR_CHECK( pwm_start() );

//   // ESP_LOGW(TAG, "GOT EMERGENCY STOP SIGNAL, DISABLING MOTORS!");
//   snprintf(message, msg_len, "GOT EMERGENCY STOP SIGNAL, DISABLING MOTORS!");
//   status_msg.data = message;
//   status_pub.publish(&status_msg);
// }

// void e_recover( const std_msgs::Empty& msg )
// {
//   int msg_len = 100;
//   char message[msg_len];

//   emergency_stop_active = false;

//   for ( int i = 0; i < 4; i++){       //Ensure pwm=0 at start
//     duties[i] = 0;                    //Set all PWM to 0
//     gpio_set_level(gpio_pins[i],1);   //Disable all low side switches
//   }

//   gpio_set_level(GPIO_NUM_15,1);      //Enable main relay
  
//   ESP_ERROR_CHECK( pwm_set_duties(duties) );
//   ESP_ERROR_CHECK( pwm_start() );
  
//   snprintf(message, msg_len, "GOT EMERGENCY RECOVER SIGNAL, ENABLING MOTORS");
//   status_msg.data = message;
//   status_pub.publish(&status_msg);
// }

ros::Subscriber<std_msgs::Int16> w_left_sub("/roboy/pinky/middleware/espchair/wheels/left", &pwm_update_L);
ros::Subscriber<std_msgs::Int16> w_right_sub("/roboy/pinky/middleware/espchair/wheels/right", &pwm_update_R);
// ros::Subscriber<std_msgs::Empty> emergency_stop_sub("/roboy/pinky/middleware/espchair/emergency/stop", &e_stop);
// ros::Subscriber<std_msgs::Empty> emergency_recover_sub("/roboy/pinky/middleware/espchair/emergency/recover", &e_recover);


// void timer_callback(void *arg)
// {
//   ESP_LOGI(TAG, "Timer expired!");
//   duties[0] = 0;
//   duties[1] = 0; 
//   for ( int i = 0; i < 4; i++){       //Stop all motors
//     gpio_set_level(gpio_pins[i],1);   //Disable all low side switches
//   }

//   ESP_ERROR_CHECK( pwm_set_duties(duties) );
//   ESP_ERROR_CHECK( pwm_start() );
// }

void rosserial_setup()
{
  // const esp_timer_create_args_t timer_cfg = {
  //   callback : &timer_callback,
  //   name : "wdt_pwm"
  // };

  nh.initNode();
  nh.advertise(status_pub);
  nh.subscribe(w_right_sub);
  nh.subscribe(w_left_sub);
  // nh.subscribe(emergency_stop_sub);
  // nh.subscribe(emergency_recover_sub);

  // ESP_ERROR_CHECK( esp_timer_init() );

  // ESP_ERROR_CHECK( esp_timer_create(&timer_cfg, &timer_handle) );


}

bool rosserial_spinonce()
{
  // int msg_len = 100;
  // char message[msg_len];
  // snprintf(message, msg_len, "WiFi connected: %d", WIFI_CONNECTED);
  // printf(message);
  // status_msg.data = message;
  // status_pub.publish(&status_msg);

  bool connected = true;
  counter++;

  if (counter % 100) {
    // ESP_LOGI(TAG, "duties0: %d, duties1: %d, ",duties[0],duties[1]);
    // ESP_LOGI(TAG, "counter: %d", counter);
  }

  if ((counter - last_left_cmd_counter) > 300) {
      ESP_LOGI(TAG, "Timer for the left motor expired!");
      duties[1] = 0; 
      duties[2] = 0;
      // gpio_set_level(GPIO_EN1_L,0);
      // gpio_set_level(GPIO_EN2_L,0);

      ESP_ERROR_CHECK( pwm_set_duties(duties) );
      ESP_ERROR_CHECK( pwm_start() );
      // connected = false;
  } 
  // else {
  //     duties[1] = 20; 
  //     duties[2] = 20;
  //     // gpio_set_level(GPIO_EN1_L,0);
  //     // gpio_set_level(GPIO_EN2_L,0);

  //     ESP_ERROR_CHECK( pwm_set_duties(duties) );
  //     ESP_ERROR_CHECK( pwm_start() );
  // }

  if ((counter - last_right_cmd_counter) > 300) {
      ESP_LOGI(TAG, "Timer for the right motor expired!");
      duties[0] = 0;
      
      ESP_ERROR_CHECK( pwm_set_duties(duties) );
      ESP_ERROR_CHECK( pwm_start() );
      // connected = false;
  }

  if (counter > 0 && (counter - last_right_cmd_counter) > 500 && (counter - last_left_cmd_counter) > 500) {
    connected = false;
  }

  nh.spinOnce();
  return connected;
}

