#include "GPIO.hh"

// STL
#include <iostream>
#include <chrono>

#include <unistd.h> // usleep()

using namespace std::chrono;
using namespace std;


class Handler
{
public:
	Handler(int id):_g(id, GPIO::Type::BOTH, std::bind(&Handler::handle, this, std::placeholders::_1, std::placeholders::_2)){
		_accum = std::chrono::duration<double, std::micro>(0.0);
		_value = _g.getValue();
	}
   void handle(unsigned short id, GPIO::Value val)
   {
	   _value = val;
      const high_resolution_clock::time_point end = high_resolution_clock::now();

      const auto time_span = duration_cast<microseconds>(end - _beg);
      _accum += time_span;
      std::cout << "id: " << id << "; Val: " << val << "; Latency: " << time_span.count() << " microseconds" << std::endl;
   }

   void startTimeout(){
	   _beg = high_resolution_clock::now();
   }

   void waitValueISR(GPIO::Value v){
   	auto timeout_us = high_resolution_clock::now() + std::chrono::microseconds(1000000);
   	while(_value != v && timeout_us > high_resolution_clock::now()){
   		//cout << "_v: " << h._value << "; v: " << v << endl;
   		usleep(10000);
   	}


   	if(_value != v){
   		cout << "Timeout waiting " << v << " on " << _g.id() << endl;
   		throw std::runtime_error("Timeout waiting " + std::to_string(v));
   	}else
   		cout << "WaitValueISR " << _g.id() << " Got: " << v << endl;
   }

   void showValue(GPIO &g1){
		//return ;
		//auto v1 = GPIO::Value::LOW;
		auto v1 = g1.getValue();
		auto v2 = _g.getValue();
		 //if (v1 != v2)
			 cout << "Set " << v1 << "; Read " << v2 << endl;
   }

protected:
   GPIO _g;
   high_resolution_clock::time_point _beg;
   duration<double, std::micro> _accum;

   volatile int _value = -1;
};

int main()
{

   {
      // Short GPIO 15 (input) to GPIO 27 (output) for the following latency test
      GPIO gpio1(395);
      GPIO gpio2(401);
      Handler h1(384);
      Handler h2(385);
      Handler h3(386);
      Handler h4(387);
      Handler h5(388);
      Handler h6(389);
      Handler h7(390);
      Handler h8(391);
      Handler h9(392);
      Handler h10(393);
      Handler h11(394);
      Handler h12(396);
      Handler h13(397);

      usleep(125000);

      gpio1.setValue(GPIO::LOW);
      usleep(125000);
      gpio1.setValue(GPIO::HIGH);
      usleep(125000);
      gpio1.setValue(GPIO::LOW);
      usleep(125000);

      cout << "Main Cycle" << endl;

      /*try {
    	  throw std::runtime_error("hello there!");
      }catch(...){

    	  cout << "Handled1 " << endl;
      }*/

      const unsigned int nIterations = 50000;
      for(unsigned int i=0;i<nIterations;++i)
      {
    	  cout << "High" << endl;

         gpio1.setValue(GPIO::HIGH);
         gpio2.setValue(GPIO::HIGH);

         h12.startTimeout();
         h8.startTimeout();
         h12.waitValueISR(GPIO::LOW);
         h8.waitValueISR(GPIO::HIGH);

         h12.showValue(gpio1);
         h8.showValue(gpio1);

         cout << "Low" << endl;

         gpio1.setValue(GPIO::LOW);
         gpio2.setValue(GPIO::LOW);

         h8.startTimeout();
         h12.startTimeout();
         h8.waitValueISR(GPIO::LOW);
         h12.waitValueISR(GPIO::HIGH);

         h12.showValue(gpio1);
         h8.showValue(gpio1);

      }

      //std::cout << "Average: " << _accum.count()/nIterations << " microseconds " << std::endl;
   }
}
