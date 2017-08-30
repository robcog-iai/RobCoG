#include "ListButton.h"
#include "Widgets/GraspTypeWidget/GraspTypeWidget.h"

UListButton::UListButton()
{
	SetBackgroundColor(FLinearColor(255,255,255,0.5));
	IsFocusable = false;

	OnClicked.AddDynamic(this, &UListButton::OnClick);
}

void UListButton::OnClick()
{
	UGraspTypeWidget* Widget = Cast<UGraspTypeWidget>(ParentWidget);
	if(Widget)
	{
		Widget->ReactToButtonClick(this->GraspType);
		//Widget->Toggle();
	}
}

void UListButton::SetupButton(UUserWidget* Widget,EGraspType GraspType)
{
	this->ParentWidget = Widget;
	this->GraspType = GraspType;
}

void UListButton::OnHovered()
{
	this->SetBackgroundColor(FLinearColor(FColor::White));
}