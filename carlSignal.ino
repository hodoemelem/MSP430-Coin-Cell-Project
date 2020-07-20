/* Author : Henry Ugochukwu Odoemelem
 * Created : 20-07-2020
 * Description : Code to provide light source and premable for coin cell project.
 */


const int LED = 13;
void setup() {
  pinMode(LED, OUTPUT);                  // initialize digital pin LED_BUILTIN as an output.
  digitalWrite(LED, HIGH);               // High
  vTaskDelay(4000 / portTICK_PERIOD_MS); // light present for 4 seconds b4 transmitting preamble  

  // start sending preamble
  digitalWrite(LED, LOW);                // LOW
  vTaskDelay(500 / portTICK_PERIOD_MS);  // wait for 0.50 sec
  digitalWrite(LED, HIGH);               // High
  vTaskDelay(450 / portTICK_PERIOD_MS);  // wait for 0.45 sec
  digitalWrite(LED, HIGH);               // High
  vTaskDelay(500 / portTICK_PERIOD_MS);  // wait for 0.50 sec
  digitalWrite(LED, LOW);                // LOW
  vTaskDelay(550 / portTICK_PERIOD_MS);  // wait for 0.50 sec
  digitalWrite(LED, HIGH);               // HIGH
  vTaskDelay(450 / portTICK_PERIOD_MS);  // wait for 0.45 sec
  digitalWrite(LED, LOW);                // LOW
  vTaskDelay(450 / portTICK_PERIOD_MS);  // wait for 0.45 sec
  digitalWrite(LED, HIGH);               // High
  vTaskDelay(550 / portTICK_PERIOD_MS);  // wait for 0.55 sec
  digitalWrite(LED, LOW);                // LOW
  vTaskDelay(500 / portTICK_PERIOD_MS);  // wait for 0.50 sec

  // keep light on after preamble
  digitalWrite(LED, HIGH);               // High
}

// the loop function runs over and over again forever
void loop() {
  
  
}
