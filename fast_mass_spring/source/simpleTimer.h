#pragma once

#include <Windows.h>
#include <iostream>

class simpleTimer
{
public:
	simpleTimer( void );
	~simpleTimer( void );

	// methods
	static void start( void );
	static void stop( std::string some_desriptor );
	
private:
	// method
	static __int64 GetTimeMs64( void );

	// member
	static __int64 start_time;
};