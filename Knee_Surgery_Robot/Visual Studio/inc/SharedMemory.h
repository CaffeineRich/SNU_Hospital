//////////////////////////////////////////////////////////////////////////////////////////////////////
// File		: SharedMemory.h
// Version	: 1.0.1
// Date		: 2022.08.31
// Writer	: Kim, YeLin (MRAS)
//////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _SHAREDMEMORY_H__
#define _SHAREDMEMORY_H__



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class Name	: CAnsySharedMemory
// Summury		: Ansynchronous Shared memory(theard unsafety)
//////////////////////////////////////////////////////////////////////////////////////////////////////////
class CAnsySharedMemory {

	// Define ////////////////////////////////////////////////////////
public:

protected:

private:
	class _CAnsySharedMemory;

	// Method ////////////////////////////////////////////////////////
public:
	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: CAnsySharedMemory
	// Input	: None
	// Output	: None
	// Summury	: Standard constructor
	////////////////////////////////////////////////////////////////////////////////////////////
	CAnsySharedMemory();



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: ~CAnsySharedMemory
	// Input	: None
	// Output	: None
	// Summury	: Standard destructor
	////////////////////////////////////////////////////////////////////////////////////////////
	~CAnsySharedMemory();



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: CreateSharedMemory
	// Input	: Memory size(int)
	// Output	: Result(bool)
	// Summury	: Create memory block.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool CreateSharedMemory(int size);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: GetSharedMemory
	// Input	: Memory pointer(void*)
	// Output	: None
	// Summury	: Copy class member memory to 'buf'.
	////////////////////////////////////////////////////////////////////////////////////////////
	void GetSharedMemory(void* buf);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: SetSharedMemory
	// Input	: Memory pointer(void*)
	// Output	: None
	// Summury	: Copy 'buf' to class member memory.
	////////////////////////////////////////////////////////////////////////////////////////////
	void SetSharedMemory(void* buf);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: GetSharedMemory
	// Input	: None
	// Output	: size of class member memory block(int)
	// Summury	: return size of memory block.
	////////////////////////////////////////////////////////////////////////////////////////////
	int GetSharedMemorySize();

protected:

private:
	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: operator =
	// Input	: src(CAnsySharedMemory&)
	// Output	: None
	// Summury	: This method is for blocking copy this class to other.
	////////////////////////////////////////////////////////////////////////////////////////////
	void operator = (CAnsySharedMemory& src) {}

	// Member ////////////////////////////////////////////////////////
public:

protected:

private:
	_CAnsySharedMemory *_poAnsyMemory;

};





//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class Name	: CSyncSharedMemory
// Summury		: Synchronous Shared memory(theard safety)
//////////////////////////////////////////////////////////////////////////////////////////////////////////
class CSyncSharedMemory {

	// Define ////////////////////////////////////////////////////////
public:

protected:

private:
	class _CSyncSharedMemory;

	// Method ////////////////////////////////////////////////////////
public:
	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: CSyncSharedMemory
	// Input	: None
	// Output	: None
	// Summury	: Standard constructor
	////////////////////////////////////////////////////////////////////////////////////////////
	CSyncSharedMemory();



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: ~CSyncSharedMemory
	// Input	: None
	// Output	: None
	// Summury	: Standard destructor
	////////////////////////////////////////////////////////////////////////////////////////////
	~CSyncSharedMemory();



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: CreateSharedMemory
	// Input	: Memory size(int)
	// Output	: Result(bool)
	// Summury	: Create memory block.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool CreateSharedMemory(int size);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: GetSharedMemory
	// Input	: Memory pointer(void*)
	// Output	: None
	// Summury	: Copy class member memory to 'buf'.
	////////////////////////////////////////////////////////////////////////////////////////////
	void GetSharedMemory(void* buf);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: SetSharedMemory
	// Input	: Memory pointer(void*)
	// Output	: None
	// Summury	: Copy 'buf' to class member memory.
	////////////////////////////////////////////////////////////////////////////////////////////
	void SetSharedMemory(void* buf);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method	: GetSharedMemory
	// Input	: None
	// Output	: size of class member memory block(int)
	// Summury	: return size of memory block.
	////////////////////////////////////////////////////////////////////////////////////////////
	int GetSharedMemorySize();

protected:

private:


	// Member ////////////////////////////////////////////////////////
public:

protected:

private:
	_CSyncSharedMemory *_poSyncMemory;
};


#endif