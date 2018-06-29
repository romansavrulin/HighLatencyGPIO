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
	Handler(int id):_g(id, GPIO::Edge::BOTH, std::bind(&Handler::handle, this, std::placeholders::_1, std::placeholders::_2)){
		_value = _g.getValue();
		_accum = std::chrono::duration<double, std::micro>(0.0);
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
   	auto timeout = 100;//00000;
   	while(_value != v && timeout--){
   		//cout << "_v: " << h._value << "; v: " << v << endl;
   		usleep(10000);
   	}


   	if(_value != v){
   		cout << "Timeout waiting " << v << "on " << _g.id() << endl;
   		throw std::runtime_error("Timeout waiting " + std::to_string(v));
   	}else
   		cout << "WaitValueISR " << _g.id() << " Got: " << v << endl;
   }

protected:
   GPIO _g;
   high_resolution_clock::time_point _beg;
   duration<double, std::micro> _accum;

   volatile int _value = GPIO::LOW;
};

void waitValuePoll(GPIO &g1, GPIO &g2){ //TODO: implement
	//return ;
	//auto v1 = GPIO::Value::LOW;
	auto v1 = g1.getValue();
	auto v2 = g2.getValue();
	 //if (v1 != v2)
		 cout << "Set " << v1 << "; Read " << v2 << endl;
}

int main()
{

   {
      // Short GPIO 15 (input) to GPIO 27 (output) for the following latency test
      GPIO gpio1(395, GPIO::Direction::OUT);
      Handler h(391);
      Handler h2(396);

      usleep(125000);

      gpio1.setValue(GPIO::LOW);
      usleep(125000);
      gpio1.setValue(GPIO::HIGH);
      usleep(125000);
      gpio1.setValue(GPIO::LOW);
      usleep(125000);

      cout << "Main Cycle" << endl;

      const unsigned int nIterations = 50000;
      for(unsigned int i=0;i<nIterations;++i)
      {
    	  cout << "High" << endl;

         gpio1.setValue(GPIO::HIGH);

         h.startTimeout();
         h2.startTimeout();
         h.waitValueISR(GPIO::HIGH);
         h2.waitValueISR(GPIO::LOW);
         //waitValuePoll(gpio1, gpio2);
         //usleep(3125000);

         cout << "Low" << endl;

         gpio1.setValue(GPIO::LOW);

         h.startTimeout();
         h2.startTimeout();
         h.waitValueISR(GPIO::LOW);
         h2.waitValueISR(GPIO::HIGH);
         //waitValuePoll(gpio1, gpio2);
         //usleep(3125000);
      }

      //std::cout << "Average: " << _accum.count()/nIterations << " microseconds " << std::endl;
   }
}
