#include "ListButton.h"
#include "Widgets/GraspTypeWidget.h"

UListButton::UListButton()
{
	SetBackgroundColor(FLinearColor(255,255,255,0.5));

	OnClicked.AddDynamic(this, &UListButton::OnClick);
}

void UListButton::OnClick()
{
	UGraspTypeWidget* Widget = Cast<UGraspTypeWidget>(ParentWidget);
	if(Widget)
	{
		Widget->ReactToButtonClick(this->GraspType);
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