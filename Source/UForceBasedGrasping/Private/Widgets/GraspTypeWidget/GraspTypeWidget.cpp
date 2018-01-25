// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "GraspTypeWidget.h"
#include "WidgetTree.h"
#include "CanvasPanelSlot.h"
#include "CanvasPanel.h"
#include "ScrollBox.h"
#include "Widgets/GraspTypeWidget/Buttons/ListButton.h"
#include "TextBlock.h"
#include "MCCharacter.h"
#include "Kismet/GameplayStatics.h"


UGraspTypeWidget::UGraspTypeWidget(const class FObjectInitializer& PCIP) : Super(PCIP)
{
	Character = nullptr;
}

void UGraspTypeWidget::SetupWidget(ACharacter* Character)
{
	this->Character = Character;
}


void UGraspTypeWidget::NativeConstruct()
{
	Super::NativeConstruct();
}

void UGraspTypeWidget::ReactToButtonClick(EGraspType GraspType)
{
	Character = UGameplayStatics::GetPlayerCharacter(GetWorld(), 0);
	AMCCharacter* MCCharacter = Cast<AMCCharacter>(this->Character);

	if(MCCharacter)
	{
		MCCharacter->SwitchGraspType(GraspType);
	}
}

TSharedRef<SWidget> UGraspTypeWidget::RebuildWidget()
{
	
	UPanelWidget* RootWidget = Cast<UPanelWidget>(GetRootWidget());

	if (RootWidget == nullptr)
	{
		RootWidget = WidgetTree->ConstructWidget<UCanvasPanel>(UCanvasPanel::StaticClass(), TEXT("RootWidget"));

		UCanvasPanelSlot* RootWidgetSlot = Cast<UCanvasPanelSlot>(RootWidget->Slot);

		if (RootWidgetSlot)
		{
			RootWidgetSlot->SetAnchors(FAnchors(0, 0, 1, 1));
			RootWidgetSlot->SetOffsets(FMargin(0, 0));
		}

		WidgetTree->RootWidget = RootWidget;
	}

	TSharedRef<SWidget> Widget = Super::RebuildWidget();

	if (RootWidget && WidgetTree)
	{
		UScrollBox* Scrollbox = WidgetTree->ConstructWidget<UScrollBox>(UScrollBox::StaticClass(), TEXT("ScrollList"));
		RootWidget->AddChild(Scrollbox);
		UCanvasPanelSlot* ScrollboxSlot = Cast<UCanvasPanelSlot>(Scrollbox->Slot);

		if (ScrollboxSlot)
		{
			ScrollboxSlot->SetAnchors(FAnchors(0, 0, 0.2, 1)); // 10% of the screen
			ScrollboxSlot->SetOffsets(FMargin(0, 0)); // Distance from top and bottom
		}

		const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EGraspType"), true);
		if (!EnumPtr) return Widget;

		for (int32 Counter = 0; Counter < EnumPtr->GetMaxEnumValue(); Counter++)
		{
			UListButton* Button = NewObject<UListButton>(this, UListButton::StaticClass());
			Button->SetupButton(this, EGraspType(Counter));

			Scrollbox->AddChild(Button);

			FText GraspTypeString = EnumPtr->GetDisplayNameTextByIndex(static_cast<int64>(Counter));
			UTextBlock* ButtonText = WidgetTree->ConstructWidget<UTextBlock>(UTextBlock::StaticClass(), TEXT("ButtonText"));
			Button->AddChild(ButtonText);
			ButtonText->SetText(GraspTypeString);
		
		}
	}
	return Widget;

}

void UGraspTypeWidget::Toggle()
{
	AMCCharacter* Character = Cast<AMCCharacter>(this->Character);
	{
		Character->ToggleUserInterface();
	}
}
