/*
The MIT License (MIT)

Copyright (c) 2014 Thomas Mercier Jr.

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

#ifndef GPIO_HH
#define GPIO_HH

#include "Uncopyable.hh"

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <fstream>
#include <memory>

// LOCKFREE define specifies the use of a (single producer, single consumer) lockfree container for
// the transfer of transition events from the thread which detects these events, to the thread which
// will call the user-provided callback function. This implementation is EXTREMELY wasteful of CPU
// time!!! However, it provides about 0.5 ms lower latency than the default implementation. If lower
// latency is required, please one of the BeagleBone Black PRUs, or a kernel module.
#ifdef LOCKFREE
   #include <boost/lockfree/spsc_queue.hpp>
#else
   #include <queue>
   #include <mutex>
   #include <condition_variable>
#endif


class GPIO : private Uncopyable
{
public:


	typedef std::shared_ptr<GPIO> sPtr;

   //-----------------------------------------------------------------------------------------------
   /// @enum Direction
   /// @brief Type used to configure a GPIO as an input or output
   //-----------------------------------------------------------------------------------------------
   enum Type {
      OUT,
      IN,
	  RISING,
	  FALLING,
	  BOTH
   };

   //-----------------------------------------------------------------------------------------------
   /// @enum Value
   /// @brief Type used to indicate the logic level of a GPIO
   //-----------------------------------------------------------------------------------------------
   enum Value {
      LOW,
      HIGH
   };


   typedef std::function<void(unsigned short, Value)> isr_callback;
   //-----------------------------------------------------------------------------------------------
   // FUNCTION NAME: Output GPIO (constructor)
   ///
   /// @brief Construt an output GPIO object.
   ///
   /// @param[in]   id         The GPIO ID. Often referred to as "pin number".
   ///
   //-----------------------------------------------------------------------------------------------
   explicit GPIO(
      unsigned short id);


   //-----------------------------------------------------------------------------------------------
   // FUNCTION NAME: GPIO (constructor)
   ///
   /// @brief Construt an input GPIO object which will call a callback function every time a
   ///        transition of type edge occurs.
   ///
   ///
   /// @param[in]   id            The GPIO ID. Often referred to as "pin number".
   /// @param[in]   type          The type of GPIO to construct.
   /// @param[in]   isr           The function to call when transitions of type RISING, FALLING or BOTH occurs. can be null.
   /// @param[in]   usePollValue  On some platforms ISR detector is doing bananas, so utilize pollValue routine instead
   ///
   /// @note If function isr throws an exception, IT WILL BE CONSUMED BY THIS CLASS.
   ///
   //-----------------------------------------------------------------------------------------------
   explicit GPIO(
      unsigned short id,
      Type type,
	  isr_callback isr,
	  bool usePollValue = true);


   //-----------------------------------------------------------------------------------------------
   // FUNCTION NAME: GPIO (destructor)
   ///
   /// @note If a program which uses this class is ungracefully terminated (this destructor is not
   ///       called), the GPIO will be left in an exported state which will prevent subsequent
   ///       construction of a GPIO object with the same id.
   //-----------------------------------------------------------------------------------------------
   ~GPIO();


   //-----------------------------------------------------------------------------------------------
   // FUNCTION NAME: setValue
   ///
   /// @brief Set the logical value (HIGH or LOW) of the GPIO. All GPIOs are active-high.
   ///
   /// @param[in]   value    The logical value to set.
   ///
   /// @return None
   ///
   //-----------------------------------------------------------------------------------------------
   void  setValue(const Value value);


   //-----------------------------------------------------------------------------------------------
   // FUNCTION NAME: getValue
   ///
   /// @brief Get the logical value (HIGH or LOW) of the GPIO. All GPIOs are active-high.
   ///
   /// @return The logical value of the GPIO.
   ///
   //-----------------------------------------------------------------------------------------------
   Value getValue();

   unsigned short id() const;

protected:
   void initCommon();
   void pollIsrLoop();
   void pollValueLoop();
   void eventDispatcherLoop();

   GPIO::Value charToValue(char c);

   GPIO::Value getValueFromSysfs();

protected:

   std::string _value_filename;

   std::fstream sysfs_value;

   Value _value;
   bool  _usePollValue;

   static const std::string  _sysfsPath;

   const unsigned short _id;
   const std::string    _id_str;
   const Type      _type;

   const isr_callback _isr;

   std::thread _pollThread;
   int _pollFD;

   std::thread _eventDispatcherThread;

   std::atomic<bool> _destructing;
   int               _pipeFD[2];

   static constexpr int _default_ownership_wait_timeout = 5000;
   static constexpr const char* _default_ownership_user = "root";
   static constexpr const char* _default_ownership_group = "gpio";

#ifdef LOCKFREE
   boost::lockfree::spsc_queue<Value, boost::lockfree::capacity<64>> _spsc_queue;
#else
   std::queue<Value>        _eventQueue; // stores values generated by interrupts
   std::mutex               _eventMutex;
   std::mutex               _sysfsValueMutex;
   std::condition_variable  _eventCV;
#endif

};

#endif
