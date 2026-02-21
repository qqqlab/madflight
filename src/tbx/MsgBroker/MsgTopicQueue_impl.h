/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

//multi publisher, multi subscriber - no fifo - locks - uses FreeRTOS Queue for storage

template <class T>
class MsgTopicQueue : public MsgTopicBase {
  private:
    QueueHandle_t queue = nullptr;
    uint32_t generation = 0;

  public:
    MsgTopicQueue(const char* name, int len) : MsgTopicBase(name) {
      queue = xQueueCreate(1, len);
    }

    uint32_t get_generation() override {
        return generation;
    }

    uint32_t publish(void *msg) override {
      xQueueOverwrite(queue, msg);
      generation++;
      return generation;
    }

    void publishFromISR(void *msg) {
      BaseType_t higherPriorityTaskWoken;
      xQueueOverwriteFromISR(queue, msg, &higherPriorityTaskWoken);
      (void)higherPriorityTaskWoken;
      generation++;
    }

    bool pull_next(void* msg, uint32_t *subscriber_gen) override {
      uint8_t tries = 5;
      uint32_t topic_gen;
      do {
        topic_gen = generation; //copy the current topic generation (generation is volatile)
        if(xQueuePeek(queue, msg, 0) != pdPASS) return false; //exit if nothing in the queue
        if(topic_gen == generation) { //if generation did not change, then we got the message for that generation
          *subscriber_gen = topic_gen; //update the subscriber generation
          return true;
        }
      }while(--tries);
      //give up, return guess of retrieved generation
      *subscriber_gen = topic_gen; //update the subscriber generation
      return true;
    }
};
