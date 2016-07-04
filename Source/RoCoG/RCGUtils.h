#pragma once
#include "RoCoG.h"
#include "RCGUtils.generated.h"


/** Enum indicating the finger type */
UENUM(BlueprintType)
enum class ERCGHandLimb : uint8
{
	Thumb		UMETA(DisplayName = "Thumb"),
	Index		UMETA(DisplayName = "Index"),
	Middle		UMETA(DisplayName = "Middle"),
	Ring		UMETA(DisplayName = "Ring"),
	Pinky		UMETA(DisplayName = "Pinky"),
	Palm		UMETA(DisplayName = "Palm")
};


/** Utils */
USTRUCT()
struct FRCGUtils
{
GENERATED_USTRUCT_BODY()

	// Default constructor	
	FRCGUtils()
	{
	};

	// Destructor
	~FRCGUtils()
	{
	};

	// TODO UTIL, get the enum value to string
	// https://wiki.unrealengine.com/Enums_For_Both_C%2B%2B_and_BP#Get_Name_of_Enum_as_String
	template<typename TEnum>
	static FORCEINLINE FString GetEnumValueToString(const FString& Name, TEnum Value)
	{
		const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, *Name, true);
		if (!EnumPtr)
		{
			return FString("Invalid");
		}

		return EnumPtr->GetEnumName((int32)Value);
	};
};
