

// defining I/O pins
#define t1out 1  // T1 digital out pin
#define t2in 2   // T2 digital in pin
#define t3in 3   // T3 digital in pin
#define t4in 4   // T4 analogue in pin
#define t4out 5 // T4 LED pin out
#define t7in 7   // T7 digital in pin
#define t7out 8  // T7 digital out pin

// variables for timing using micros()
unsigned long tm;
unsigned long newtm;
QueueHandle_t buttonEventQueue;
SemaphoreHandle_t mutex;
// freqency calculation on Task2 & 3 square waves
volatile unsigned long freqhigh;
volatile int freq1;
volatile int freq2;
int mapf1;
int mapf2;

// Task4 avg analogue value calculation
int a[10];
volatile int counter = 0;
int avg = 0;

char buffer[100];
int buffer_flag = 0;
int asum=0;

void Task1(void *pvParameters)
{
    while (1)
    {
        // Task1 pulse sequence
        digitalWrite(t1out, HIGH);
        delayMicroseconds(180);
        digitalWrite(t1out, LOW);
        delayMicroseconds(40);
        digitalWrite(t1out, HIGH);
        delayMicroseconds(530);
        digitalWrite(t1out, LOW);
        delayMicroseconds(250);
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }
}

int pulse_m(int pin, int state, int timeout)
{
    tm = micros();
    while (digitalRead(pin) != state)
    {
        if (micros() - tm > timeout)
        {
            return 0;
        }
    }
    newtm = micros();
    while (digitalRead(pin) == state)
    {
        if (micros() - newtm > timeout)
        {
            return 0;
        }
    }
    return micros() - tm;
}

void Task2(void *pvParameters) {
    while (1) {
        xSemaphoreTake(mutex, portMAX_DELAY);  // 请求互斥锁
        freqhigh = pulseIn(t2in, HIGH, 3000);
        freq1 = 1000000.0 / (freqhigh * 2.0);
        if (freqhigh == 0) {
            freq1 = 0;
        }
        xSemaphoreGive(mutex);  // 释放互斥锁
        vTaskDelay(20 / portTICK_PERIOD_MS);  // rate 50hz
    }
    vTaskDelete(NULL);
}

void Task3(void *pvParameters) {
    while (1) {
        xSemaphoreTake(mutex, portMAX_DELAY);  // 请求互斥锁
        freqhigh = pulseIn(t3in, HIGH, 3000);
        freq2 = 1000000.0 / (freqhigh * 2.0);
        if (freqhigh == 0) {
            freq2 = 0;
        }
        xSemaphoreGive(mutex);  // 释放互斥锁
        vTaskDelay(8 / portTICK_PERIOD_MS);  // rate 125hz
    }
    vTaskDelete(NULL);
}

void Task4(void *pvParameters)
{
    while (1)
    {
        // counter 0-9
        if (counter > 9)
        {
            counter = 0;
        }
        // read analogue signal on pin
        a[counter] = analogRead(t4in);
        // average value
        for(int k = 0 ; k < 10 ; k++){
          asum=asum+k;
        }
        avg = asum / 10;
        asum = 0;
        counter++;

        // LED high if above 2047 (max 4096)
        if (avg > 2047)
        {
            digitalWrite(t4out, HIGH);
        }
        else
        {
            digitalWrite(t4out, LOW);
        }
        // rate 50hz
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void CPU_work(int time)
{
    for (int i = 0; i < time; i++)
    {
        // CPU work,delay 1ms
        delayMicroseconds(1000);
    }
}


void somePeriodicTask(void *pvParameters) {
    while (1) {
        CPU_work(2);  // 让CPU忙碌大约2毫秒
        vTaskDelay(pdMS_TO_TICKS(20) - pdMS_TO_TICKS(2));  // 总周期20ms，已消耗2ms
    }
}

void Task5(void *pvParameters)
{
    // Task5 print T2 & 3 values
    // HardwareSerial *Serial = (HardwareSerial *)pvParameters;
    // 10hz
    while (1)
    {
        // setting outliers of freq range as 0 or 99
        if(freq1<333)
        {
          mapf1=0;
        }else if(freq1>1000)
        {
          mapf1=99;
        }else
        {
          mapf1=(freq1-333)/667*99
        }
        if(freq2<500)
        {
          mapf2=0;
        }else if(freq2>1000)
        {
          mapf2=99;
        }else
        {
          mapf2=(freq2-500)/500*99
        }

        // print T2 & 3 values
        sprintf(buffer, "T2: %d , T3: %d \n", mapf1, mapf2);
        buffer_flag = 1;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void buttonMonitorTask(void *pvParameters) {
    int buttonState = 0;
    int lastButtonState = 0;
    vTaskDelay(30 / portTICK_PERIOD_MS);
    while (1) {
        buttonState = digitalRead(t7in);  // 读取按钮状态
        if (buttonState != lastButtonState && buttonState == 1) {
            // 按钮状态从未按压变为按压，发送事件到队列
            vTaskDelay(30 / portTICK_PERIOD_MS);
            xQueueSend(buttonEventQueue, &buttonState, portMAX_DELAY);
        }
        lastButtonState = buttonState;
        vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms debounce delay
    }
}

void ledControlTask(void *pvParameters) {
    int receivedState;

    while (1) {
        if (xQueueReceive(buttonEventQueue, &receivedState, portMAX_DELAY) == pdPASS) {
            // 从队列接收到事件，控制LED
            digitalWrite(t7out, receivedState ? HIGH : LOW);
        }
    }
}

void setup()
{
    // // set baud rate 9600
    Serial.begin(115200);

    mutex = xSemaphoreCreateMutex();  // 创建互斥锁
    // // initialise output pins for Task1 & 4
    pinMode(t1out, OUTPUT);
    pinMode(t4out, OUTPUT);
    pinMode(t7out, OUTPUT);

    // // initialise input pins for Task2, 3 & 4
    pinMode(t2in, INPUT);
    pinMode(t3in, INPUT);
    pinMode(t4in, INPUT);
    pinMode(t7in, INPUT);

    // initialise an extra pin 38 for output pwm
    // 700hz and duty = 50%
    ledcSetup(0, 700, 8);
    ledcAttachPin(38, 0);
    ledcWrite(0, 128);

    // // create tasks
    xTaskCreate(Task1, "Task1", 4096, NULL, 3, NULL);
    xTaskCreate(Task2, "Task2", 4096, NULL, 2, NULL);
    xTaskCreate(Task3, "Task3", 4096, NULL, 2, NULL);
    xTaskCreate(Task4, "Task4", 4096, NULL, 2, NULL);
    xTaskCreate(Task5, "Task5", 4096, NULL, 2, NULL);
    xTaskCreate(buttonMonitorTask, "Button Monitor", 1000, NULL, 1, NULL);
    xTaskCreate(ledControlTask, "LED Control", 1000, NULL, 1, NULL);
    xTaskCreate(somePeriodicTask, "Periodic Task", 2048, NULL, 1, NULL);
      // 创建队列，长度为10，元素大小为int
    buttonEventQueue = xQueueCreate(10, sizeof(int));


    // 启动调度器
    vTaskStartScheduler();

}

void loop()
{
    vTaskDelay(1 / portTICK_PERIOD_MS);
    if (buffer_flag == 1)
    {
        Serial.print(buffer);
        buffer_flag = 0;
    }
    // printf("loop task\n");
}


