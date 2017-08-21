#include "ListButton.h"
#include "Widgets/GraspTypeWidget.h"


UListButton::UListButton()
{
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
