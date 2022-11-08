
/*
* microros_tasks.c

* Useful commands in ros2 env.
    * ros2 run micro_ros_setup create_agent_ws.sh
    * ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyUSB0


********************************************************************************
*
* Created on: March 15, 2022
*     Author: Nipun Dhananjaya Weerakkodi -> nipun.dhananjaya@gmail.com
********************************************************************************
*/


#include "microros_tasks.h"


/* ====================================== Sub Defines ====================================== */


// LED error indicator
void error_blink(uint8_t n){
    for (int i = 0; i < n; i++){
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET); // red
        HAL_Delay(100);
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET); // red
        HAL_Delay(100);
    }
    HAL_Delay(1000);
}

// ========================   microros_config   ==================================
void hypergod_microros_init(void){
    MX_USART2_UART_Init();
    rmw_ret_t status = rmw_uros_set_custom_transport(
                            true,
                            (void *) &huart2,
                            cubemx_transport_open,
                            cubemx_transport_close,
                            cubemx_transport_write,
                            cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__); 
        }
}


// =============================  NODE  ==================================
// create node ---------------------
void init_hyperdogSTM_node(void)
{
    //create init_options
    allocator = rcl_get_default_allocator();
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
   
    // wait till connect to agent
    if (rc != RCL_RET_OK){
        while (1){   
            // blink 3 times
            error_blink(1);
            rclc_support_fini(&support);
            rc = rclc_support_init(&support, 0, NULL, &allocator);
            if (rc == RCL_RET_OK){
                break;
            }
            
        }
    }
    // create node
    rcl_ret_t rc2 = rclc_node_init_default(&node, "Hyperdog_STM", "", &support);
    
}


// =========================  PUBLISHER  ================================
// create publisher topic ---------------------
void create_hyperdogSTM_publisher(void)
{
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "hyperdogSTM_publisher"));
    // data type configure and set malloc for sequence msg
    pub_msg.data.capacity = 100;
    pub_msg.data.data = (float *) malloc(pub_msg.data.capacity * sizeof(float));

    
    pub_msg.data.size = 12;
    for (int i = 0; i < pub_msg.data.size; i++){
        pub_msg.data.data[i] = 0.0;
    }
}

// publisher callback -------------------
void publisher_callback(rcl_timer_t * timer, int64_t last_call_time){
    (void) last_call_time;
    (void) timer;

    for (int i = 0; i < pub_msg.data.size; i++){
        pub_msg.data.data[i] = joint_ang[i];
    }
    uint8_t err_count = 0;
    rcl_ret_t rc = rcl_publish(&publisher, &pub_msg, NULL);
    if (rc != RCL_RET_OK){
        error_blink(2);
        run_hyperdogSTM_node();
    }
    HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
}


// =========================  SUBSCRIBER  ================================
// create subscriber ---------------------
void create_hyperdogSTM_subscriber(void)
{   
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "hyperdog_jointController/commands")); 
    // data type configure and set malloc for sequence msg
    sub_msg.data.capacity = 100;
    sub_msg.data.data = (float *) malloc(pub_msg.data.capacity * sizeof(float));
    sub_msg.data.size = 12;
    // initialize servo driver PCA9685
    bool err = PCA9685_Init(&hi2c3);
    if (err){
        while (err ){
            err = PCA9685_Init(&hi2c3);
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET); // red LED
            HAL_Delay(100);
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET); // red LED
            HAL_Delay(1000);
            
        }
    }
}

// subscriber callback ---------------------
void subscription_callback(const void * msgin){
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET); // Green LED on
    // Cast received message to used type
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    // proccess msg
    for (int i = 0; i<12; i++){
        joint_ang[i] = msg->data.data[i];
    }
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET); // Green LED off
}



/* ========================================================================================= */
/* ====================================== Main Define ====================================== */
/* ========================================================================================= */

void run_hyperdogSTM_node(void)
{   
    // initial configurations 
    hypergod_microros_init();
    init_hyperdogSTM_node();
    create_hyperdogSTM_publisher();
    create_hyperdogSTM_subscriber();

    // timer for publisher
    rcl_timer_t timer_pub;
    RCCHECK(rclc_timer_init_default(
        &timer_pub,
        &support,
        RCL_MS_TO_NS(5),
        publisher_callback));

    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_pub));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));
    

    // spin node by executor 
    while(1){
       
        rcl_ret_t rc = rclc_executor_spin(&executor); //, RCL_MS_TO_NS(100)
        if (rc != RCL_RET_OK){
            break;
        }
    }
    // rclc_executor_fini(&executor);

}

