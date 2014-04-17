#pragma once

#include <Windows.h>
#include <iostream>

class simpleTimer
{
public:
	simpleTimer( void );
	~simpleTimer( void );

	// methods
	void start( void );
	void stop( std::string some_desriptor );
	
private:
	// method
	__int64 GetTimeMs64( void );

	// member
	__int64 start_time;
};