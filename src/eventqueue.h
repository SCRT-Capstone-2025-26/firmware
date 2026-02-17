#ifndef EVENTQUEUE_H
#define EVENTQUEUE_H
/*
MIT License

Copyright (c) 2018 Jukka-Pekka Sarjanen

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
*/

// This has been modified to use a semaphore and allow blocking
// Currently a fairly poor implementation, but it should work

#include <inttypes.h>
#include <pico/sem.h>

///Array based thread safe queue for Arduino.
/// Implement FIFO type queue that use array as ringbuffer
/// to store messages.
///
template <class T, uint8_t s=8>
class EventQueue{
public:
    EventQueue(): _in(0), _out(0), _count(0), _s(s) {
        sem_init(&_rw_sem, 1, 1);
        sem_init(&_count_sem, 0, s);
    }

    // Pop first item from queue.
    bool getQ(T &e, bool blocking) {
        if (blocking) {
            sem_acquire_blocking(&_count_sem);
        } else {
            if (!sem_try_acquire(&_count_sem)) {
              return false;
            }
        }

        sem_acquire_blocking(&_rw_sem);
        if (_count > 0) {
            _count--;

            e = _queue[_out%_s];
            _out = (++_out/_s)?0:_out;
        }
        sem_release(&_rw_sem);

        return true;
    }

    // Insert item to queue.
    bool putQ(const T &&e){
        bool rc = false;

        sem_acquire_blocking(&_rw_sem);

        if (_count < _s){
            _count++;
            sem_release(&_count_sem);

            _queue[_in%_s] = e;
            _in =  (++_in/_s)?0:_in;
            rc = true;
        }
        sem_release(&_rw_sem);

        return rc;
    }

private:
    // Old stuff
    uint8_t _in;
    uint8_t _out;
    // This is the actual number of items in the queue
    //  and should only be change when the thread has the rw lock
    uint8_t _count;
    const    uint8_t _s;
    T       _queue[s];

    // New
    // _rw_sem is a lock on the the structure
    semaphore_t _rw_sem;
    // _count_sem essentially represents the availablility to "claim" a read
    // If you have this semaphore you are permitted to do a read if you don't you cannot
    // This means that _count_sem must be less than ore equal to _count to prevent
    // authorizing a read without enough values in the queue
    // If no threads have "claimed" a read then this is equal to _count
    // This allows a blocking acquire on this to function as a blocking read
    semaphore_t _count_sem;
};

#endif // EVENTQUEUE
