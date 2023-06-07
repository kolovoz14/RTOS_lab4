
# RTOS_lab4-Stream Buffers experiments

Scenario:
* a thread called monitor (through UART or display)
  * should be implemented using StreamBuffer mechanism
      * trigger level = 1
  * other threads have to use xStreamBufferSend()
  * monitor uses xStreamBufferReceive() with some small timeout
* ADC should constantly work
  * only one analog channel has to be sampled
  * any analog channel can be chosen
  * inside the ISR caused by EOF:
    * a sample has to be taken, and if the value of the sample is higher than some threshold then the message (i.e. character string) should be sent to the StreamBuffer using xStreamBufferSendFromISR()
    * one can set up a new conversion at the end of the ISR callback, or any hardware TIM can be utilized

* There should exist two other threads/tasks being reading a value from some sensors placed on the same I2C/SPI bus. Therefore, both threads have to compete for access to a bus. So, the bus driver must implement mutex/semaphore protection.
  * sensors can be chosen among given by a teacher
  * in case of a lack of enough sensors, both threads can take data from the same sensor (i.e. the only one)


## Authors

- [@kolovoz14](https://github.com/kolovoz14)

