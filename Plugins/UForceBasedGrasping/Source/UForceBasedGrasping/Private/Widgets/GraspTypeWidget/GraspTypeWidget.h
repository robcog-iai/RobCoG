// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#pragma once

//#include "Buttons/ListButton.h"
#include "Blueprint/UserWidget.h"
#include "SWidget.h"


#include "GraspTypeWidget.generated.h"

enum class EGraspType : unsigned char;

/**
 * This class deals with the grasping of a hand
 */
UCLASS()
class UFORCEBASEDGRASPING_API UGraspTypeWidget : public UUserWidget
{
	GENERATED_BODY()

public:

	UGraspTypeWidget(const class FObjectInitializer& PCIP);
	virtual void NativeConstruct() override;
	virtual TSharedRef<SWidget> RebuildWidget() override;

	void SetupWidget(ACharacter* Character);
	void ReactToButtonClick(EGraspType GraspType);

	void Toggle();



private:
	ACharacter* Character;
	
};
