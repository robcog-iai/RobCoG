#pragma once

#include "CoreMinimal.h"
#include "Components/Button.h"
#include "Enums/GraspType.h"

#include "ListButton.generated.h"

/*
*
*/
UCLASS()
class UFORCEBASEDGRASPING_API UListButton : public UButton
{
	GENERATED_BODY()

private:
	UPROPERTY()
		EGraspType GraspType;

	UUserWidget* ParentWidget;

public:
	UListButton();

	UFUNCTION()
		void OnClick();
	
	UFUNCTION()
		void OnHovered();

	void SetupButton(UUserWidget* Widget, EGraspType GraspType);
};
