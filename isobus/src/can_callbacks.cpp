//================================================================================================
/// @file can_callbacks.cpp
///
/// @brief An object to represent common callbacks used within this CAN stack.
/// @author Adrian Del Grosso
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================
#include "isobus/isobus/can_callbacks.hpp"

namespace isobus
{
	ParameterGroupNumberCallbackData::ParameterGroupNumberCallbackData(std::uint32_t parameterGroupNumber, CANLibCallback callback, void *parentPointer, InternalControlFunction *destinationFunction) :
	  mCallback(callback),
	  mParameterGroupNumber(parameterGroupNumber),
	  mParent(parentPointer),
	  mDestinationFunction(destinationFunction)
	{
	}

	ParameterGroupNumberCallbackData::ParameterGroupNumberCallbackData(const ParameterGroupNumberCallbackData &oldObj)
	{
		mCallback = oldObj.mCallback;
		mParameterGroupNumber = oldObj.mParameterGroupNumber;
		mParent = oldObj.mParent;
		mDestinationFunction = oldObj.mDestinationFunction;
	}

	bool ParameterGroupNumberCallbackData::operator==(const ParameterGroupNumberCallbackData &obj)
	{
		return ((obj.mCallback == this->mCallback) &&
		        (obj.mParameterGroupNumber == this->mParameterGroupNumber) &&
		        (obj.mParent == this->mParent) &&
		        (obj.mDestinationFunction == this->mDestinationFunction));
	}

	ParameterGroupNumberCallbackData &ParameterGroupNumberCallbackData::operator=(const ParameterGroupNumberCallbackData &obj)
	{
		mCallback = obj.mCallback;
		mParameterGroupNumber = obj.mParameterGroupNumber;
		mParent = obj.mParent;
		mDestinationFunction = obj.mDestinationFunction;
		return *this;
	}

	std::uint32_t ParameterGroupNumberCallbackData::get_parameter_group_number() const
	{
		return mParameterGroupNumber;
	}

	CANLibCallback ParameterGroupNumberCallbackData::get_callback() const
	{
		return mCallback;
	}
	
	InternalControlFunction *ParameterGroupNumberCallbackData::get_destination_control_function() const
	{
		return mDestinationFunction;
	}

	void *ParameterGroupNumberCallbackData::get_parent() const
	{
		return mParent;
	}

	ControlFunctionStatusUpdateCallbackData::ControlFunctionStatusUpdateCallbackData(ControlFunction *controlFunction, ControlFunctionStatusUpdateCallback callback, void *parentPointer) :
	  mCallback(callback),
	  mControlFunction(controlFunction),
	  mParent(parentPointer)
	{
	}

	ControlFunctionStatusUpdateCallbackData::ControlFunctionStatusUpdateCallbackData(const ControlFunctionStatusUpdateCallbackData &oldObj)
	{
		mCallback = oldObj.mCallback;
		mControlFunction = oldObj.mControlFunction;
		mParent = oldObj.mParent;
	}

	bool ControlFunctionStatusUpdateCallbackData::operator==(const ControlFunctionStatusUpdateCallbackData &obj)
	{
		return ((obj.mCallback == this->mCallback) &&
		        (obj.mControlFunction == this->mControlFunction) &&
		        (obj.mParent == this->mParent));
	}

	ControlFunctionStatusUpdateCallbackData &ControlFunctionStatusUpdateCallbackData::operator=(const ControlFunctionStatusUpdateCallbackData &obj)
	{
		mCallback = obj.mCallback;
		mControlFunction = obj.mControlFunction;
		mParent = obj.mParent;
		return *this;
	}

	ControlFunction* ControlFunctionStatusUpdateCallbackData::get_control_function() const
	{
		return mControlFunction;
	}

	ControlFunctionStatusUpdateCallback ControlFunctionStatusUpdateCallbackData::get_callback() const
	{
		return mCallback;
	}
	
	void *ControlFunctionStatusUpdateCallbackData::get_parent() const
	{
		return mParent;
	}
} // namespace isobus
