//#include <ros.h>
//#include <std_msgs/String.h>
//#include <drone_controller/AttitudeArray.h>
//#include <drone_controller/PID_Gains.h>
//#include <sstream>
//
//const char *ssid = "TEdataF3F160";
//const char *password = "Ahmed@2211";
//// Set the rosserial socket server IP address
//IPAddress server(192, 168, 1, 8);
//// Set the rosserial socket server port
//const uint16_t serverPort = 11411;
//
//std::string str = "";
//int count = 0;
//
//ros::NodeHandle nh;
//
//
//void attitude_callback(const drone_controller::AttitudeArray& msg) {
//    // Process the incoming message, for example, print it to the serial monitor
//    Serial.println(msg.data[1]);
//}
//
//void attitude_callback(const drone_controller::PID_Gains& msg) {
//    // Process the incoming message, for example, print it to the serial monitor
//    Serial.print("kp = ");
//    Serial.print(msg.kp);
//    Serial.print(", ki = ");
//    Serial.print(msg.ki);
//    Serial.print(", kd = ");
//    Serial.println(msg.kd);
//}
//
//ros::Subscriber<drone_controller::AttitudeArray> attitude_sub("/cmd_attitude", attitude_callback);
//ros::Subscriber<drone_controller::PID_Gains> pid_sub("/pid_gains", attitude_callback);
//
//// Be polite and say hello
//char hello[13] = "hello world!";
//
////void setup()
////{
////    // Use ESP8266 serial to monitor the process
////    Serial.begin(115200);
////    Serial.println();
////    Serial.print("Connecting to ");
////    Serial.println(ssid);
////
////    // Connect the ESP8266 the the wifi AP
////    WiFi.begin(ssid, password);
////    while (WiFi.status() != WL_CONNECTED) {
////        delay(500);
////        Serial.print(".");
////    }
////    Serial.println("");
////    Serial.println("WiFi connected");
////    Serial.println("IP address: ");
////    Serial.println(WiFi.localIP());
////
////    // Set the connection to rosserial socket server
////    nh.getHardware()->setConnection(server, serverPort);
////    nh.initNode();
////
////    // Another way to get IP
////    Serial.print("IP = ");
////    Serial.println(nh.getHardware()->getLocalIP());
////
////    // Start to be polite
////    nh.advertise(chatter);
////}
//
//
//// Task function for ROS logic (runs on core 0)
//[[noreturn]] void rosTask(void *pvParameters) {
//    while (true) {
//        if (nh.connected()) {
//            // Say hello
////            std::stringstream ss;
////            ss << count;
////            str = ss.str();
////            str_msg.data = str.c_str();
////            count++;
////            chatter.publish(&str_msg);
////            Serial.println(count);
//        } else {
//            Serial.println("Not Connected");
//        }
//        nh.spinOnce();
//        const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
//        vTaskDelay(xDelay);
//
//    }
//}
//
//// Task function for empty update (runs on core 1)
//[[noreturn]] void emptyLoopTask(void *pvParameters) {
//    while (true) {
//       // Serial.println("I'm free");
//        const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
//        vTaskDelay(xDelay);
//    }
//}
//
//void setup() {
//
//    Serial.begin(115200);
//    Serial.println();
//    Serial.print("Connecting to ");
//    Serial.println(ssid);
//    // Connect to WiFi network
//    WiFi.begin(ssid, password);
//    while (WiFi.status() != WL_CONNECTED) {
//        delay(500);
//        Serial.print(".");
//    }
//
//    Serial.println("");
//    Serial.println("WiFi connected");
//    Serial.println("IP address: ");
//    Serial.println(WiFi.localIP());
//
//    // Initialize ROS node handle
//    nh.getHardware()->setConnection(server, serverPort);
//    nh.initNode();
//    nh.subscribe(attitude_sub);
//    nh.subscribe(pid_sub);
//
//    xTaskCreatePinnedToCore(
//            rosTask,          // Task function
//            "rosTask",        // Task name
//            10000,            // Stack size
//            NULL,             // Task parameters
//            1,                // Priority
//            NULL,             // Task handle
//            0                 // Core number (0 for core 0)
//    );
//
//// Create empty update task on core 1
//    xTaskCreatePinnedToCore(
//            emptyLoopTask,    // Task function
//            "emptyLoopTask",  // Task name
//            10000,            // Stack size
//            NULL,             // Task parameters
//            1,                // Priority
//            NULL,             // Task handle
//            1                 // Core number (1 for core 1)
//    );
//}
//
//void update() {
//    // This update remains empty as it will run on core 1
//}
//
////void update()
////{
////
////    if (nh.connected()) {
////        Serial.println("Connected");
////        // Say hello
////        str_msg.data = hello;
////        chatter.publish( &str_msg );
////    } else {
////        Serial.println("Not Connected");
////    }
////    nh.spinOnce();
////    // Loop exproximativly at 1Hz
////    delay(1000);
////}

//void setup() {
//
//}
//
//void loop() {
//
//}