// Copyright 2017, Institute for Artificial Intelligence - University of Bremen

#include "RRobot.h"
#include "RStaticMeshComponent.h"
#include "RURDFParser.h"
#include "math.h"
#include "Engine.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Runtime/Engine/Classes/Engine/StaticMeshActor.h"
#include "UnrealEd.h"
#include "Engine/EngineTypes.h"

// Sets default values
ARRobot::ARRobot()
{
	// Init parameters for substepping in RStaticMeshComponent for which this is the parent actor
	StartVelocity = 1000.0f;
	KSpring = 100.0f;
	Damping = 0;		
	
	//Get values from UROboSimEd
	if (IURoboSimEd::IsAvailableEd()) {
		FURoboSimEdModule& placeHolder = IURoboSimEd::GetEd();
		bEnableUROSBridge = placeHolder.bEnableUROSBridge;
		bEnableShapeCollisions = placeHolder.bEnableShapeCollisions;
		bEnableCollisionTags = placeHolder.bEnableShapeCollisions;
		bEnableAngularMotors = placeHolder.bEnableAngularMotors;
		bEnableCollisionTags = placeHolder.bEnableCollisionTags;
		collisionFilterArr = placeHolder.collisionFilterArr;
	}	
		
	// Create a USceneComponent to be the RootComponent
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("DefaultSceneComponent"));
	Root->SetupAttachment(this->RootComponent, TEXT("DefaultSceneComponent"));
	this->SetRootComponent(Root);
	Root->bVisualizeComponent = true;
    UE_LOG(LogTemp, Warning, TEXT("Root - Rotation = [%s]"), *Root->GetComponentRotation().ToString()); 

	// Preload meshes so they are available in runtime
	static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshCy(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	//static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshCu(TEXT("/Game/Geometry/Meshes/1M_Cube.1M_Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshCu(TEXT("/Engine/BasicShapes/Cube.Cube"));
	//static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshSp(TEXT("/Game/StarterContent/Shapes/Shape_Sphere.Shape_Sphere")); // Pivot point is not in the center
	static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshSp(TEXT("/Engine/BasicShapes/Sphere.Sphere"));

	//
	//static ConstructorHelpers::FObjectFinder<UMaterial> Material(TEXT("/Game/StarterContent/Materials/M_Brick_Clay_New.M_Brick_Clay_New"));
	//static ConstructorHelpers::FObjectFinder<UMaterial> Material(TEXT("Material'/Game/StarterContent/Materials/M_Metal_Gold.M_Metal_Gold'"));

	if (MeshCy.Succeeded())
	{
		CylinderMesh = MeshCy.Object;
	}
	if (MeshCu.Succeeded())
	{
		CubeMesh = MeshCu.Object;
	}
	if (MeshSp.Succeeded())
	{
		SphereMesh = MeshSp.Object;
	}
	/* Material support disabled for now
	if (Material.Succeeded())
	{
		//BasicMaterial = Material.Object;
	}
	*/
}

// Called when the game starts or when spawned
void ARRobot::BeginPlay()
{
	Super::BeginPlay();

	TArray<FString> Keys;
	OriginLocations.GenerateKeyArray(Keys);

	for (FString LinkName : Keys)
	{
		UPrimitiveComponent* PrimComp = LinkComponents.FindRef(LinkName);
		// Location will be off by about 1/3 after the first gameframe so this extra value is added additionally
		FVector MovementVector(OriginLocations.FindRef(LinkName) * -1.333333333f);
		if (PrimComp)
			PrimComp->AddRelativeLocation(MovementVector, true);
		else
			UE_LOG(LogTemp, Warning, TEXT("Error correcting location of link %s\n"), LinkName.GetCharArray().GetData());
	}
	
}

// Called every frame (currently not active)
void ARRobot::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	// UE_LOG(LogTemp, Log, TEXT("Testing"));

//    if (GFrameCounter % 5 == 0)
//    {
//        // Get All Rigid Bodies Rotation
//        /*UE_LOG(LogTemp, Log, TEXT("At Frame [%d], Size of LinkComponents = [%d]"), GFrameCounter, LinkComponents.Num());
//        for (auto &LinkElement : LinkComponents)
//        {
//            UE_LOG(LogTemp, Log, TEXT("Component [%s], Location [%s], Rotation [%s]"),
//                *LinkElement.Value->GetName(), *LinkElement.Value->GetComponentLocation().ToString(),
//                *LinkElement.Value->GetComponentRotation().ToString());
//        }*/

//        for (auto &JointElement : JointComponents)
//        {
//            /* UE_LOG(LogTemp, Log, TEXT("Component [%s], Location [%s], Rotation [%s], Sw1, Sw2, Tw = [%.3f, %.3f, %.3f]"),
//                *JointElement.Value->GetName(), *JointElement.Value->GetComponentLocation().ToString(),
//                *JointElement.Value->GetComponentRotation().ToString(),
//                JointElement.Value->GetCurrentSwing1(),
//                JointElement.Value->GetCurrentSwing2(),
//                JointElement.Value->GetCurrentTwist()
//                );*/
//            GetJointPosition(JointElement.Key);
//            GetJointVelocity(JointElement.Key);
//        }
//    }
}

void ARRobot::OnConstruction(const FTransform &Transform)
{
	ParseURDF();

	// Set material for all static meshes
	if (BasicMaterial)
	{
		TArray<UActorComponent*> Children = GetComponents().Array();
		for (auto ChildComp : Children)
		{
			if (ChildComp->IsA(UStaticMeshComponent::StaticClass()))
			{
				UStaticMeshComponent* Mesh = (UStaticMeshComponent*)ChildComp;
				for (int c = 0; c < Mesh->GetNumMaterials(); c++)
				{
					Mesh->SetMaterial(c, BasicMaterial);
				}
			}
		}
	}
}

bool ARRobot::AddLink(FRLink Link)
{
	FRLink LinkToAdd(Link);
	Links.Add(LinkToAdd);
	return true;
}

bool ARRobot::AddJoint(FRJoint Joint)
{
	FRJoint JointToAdd(Joint);
	Joints.Add(JointToAdd);
	return true;
}

bool ARRobot::CreateRobot()
{
	if (Links.Num() <= 1)
	{
		return false;
	}

	// Since every Link other than the base link has to be connected to another link the number of joints is fixed
	if (Joints.Num() != Links.Num() - 1)
	{
		UE_LOG(LogTemp, Warning, TEXT("Robot is invalid. Abort creating.\n"));
		//return false;
	}

	FRJoint CurrentJoint(Joints[0]);

	// Search for upmost link
	while (true)
	{
		bool FoundHigherElement = false;
		for (int c = 0; c < Joints.Num(); c++)
		{
			if (CurrentJoint.Parent.Equals(Joints[c].Child))
			{
				CurrentJoint = Joints[c];
				FoundHigherElement = true;
			}
		}
		if (!FoundHigherElement) break;
	}

	// If there already is a tree free its memory and continue
	if (BaseNode)
	{
		delete BaseNode;
	}

	// Setup the BaseNode
	BaseNode = new FRNode();
	BaseNode->Parent = nullptr;

	for (int c = 0; c < Links.Num(); c++)
	{
		if (Links[c] == CurrentJoint.Parent)
		{
			BaseNode->Link = Links[c];
			Links.RemoveAt(c);
		}
	}

	BuildTree(BaseNode);

	bool bSuccess = CreateActorsFromNode(BaseNode);



	return bSuccess;
}

bool ARRobot::CreateActorsFromNode(FRNode* Node)
{
	if (!Node)
	{
		return false;
	}

	FRLink* Link = &(Node->Link);
	FRJoint* Joint = &(Node->Joint);
	USceneComponent* ParentComp;
	UPrimitiveComponent* ParentLink = nullptr;
	Joint->LowerLimit *= 50.f;
	Joint->UpperLimit *= 50.f;
	FVector LinkOriginLocation(0);

	bool bUseVisual = !(Link->Visual.Mesh.IsEmpty());
	bool bUseCollision = !(Link->Collision.Mesh.IsEmpty());
	
	bUseCollision = bEnableCollisionTags;
	
	if (Node->Parent)
	{
		// Parent exists and is a UMeshComponent
		ParentComp = (USceneComponent*)LinkComponents.FindRef(Node->Parent->Link.Name);
		ParentLink = LinkComponents.FindRef(Node->Parent->Link.Name);
	}
	else
	{
		// This node is the topmost so the parent has to be the RootComponent
		ParentComp = RootComponent;
	}

	// Quit if the link already exists
	if (LinkComponents.Contains(Link->Name))
	{
		return false;
	}

	UStaticMesh* Mesh = nullptr;
	//UStaticMeshComponent* MeshComp = nullptr;
	URStaticMeshComponent* MeshComp = nullptr;
	UShapeComponent* ShapeComp = nullptr;
	FVector Scale(Link->Visual.Scale);

	if ((bUseCollision && !bUseVisual) || (!bUseCollision && bUseVisual)) // Collision and Visual are the same
	{
		if (Link->Visual.Mesh.Equals("box", ESearchCase::IgnoreCase) || Link->Collision.Mesh.Equals("box", ESearchCase::IgnoreCase))
		{
			Mesh = CubeMesh;
			ShapeComp = NewObject<UBoxComponent>(Root, FName(Link->Name.GetCharArray().GetData()));
			FVector BoxSize(Scale);
			BoxSize *= 50.f;
			((UBoxComponent*)ShapeComp)->InitBoxExtent(BoxSize);
			((UBoxComponent*)ShapeComp)->SetBoxExtent(BoxSize);
		}
		else if (Link->Visual.Mesh.Equals("cylinder", ESearchCase::IgnoreCase))// || Link->Collision.Mesh.Equals("cylinder", ESearchCase::IgnoreCase))
		{
			Mesh = CylinderMesh;
			ShapeComp = NewObject<UCapsuleComponent>(Root, FName(Link->Name.GetCharArray().GetData()));
			float Radius = Scale.X * 50.f;
			float Height = Scale.Z * 50.f;
			((UCapsuleComponent*)ShapeComp)->InitCapsuleSize(Radius, Height);
			((UCapsuleComponent*)ShapeComp)->SetCapsuleSize(Radius, Height);
		}
		else if (Link->Visual.Mesh.Equals("sphere", ESearchCase::IgnoreCase) || Link->Collision.Mesh.Equals("sphere", ESearchCase::IgnoreCase))
		{
			Mesh = SphereMesh;
			ShapeComp = NewObject<USphereComponent>(Root, FName(Link->Name.GetCharArray().GetData()));
			float Radius = Scale.X * 50.f;
			((USphereComponent*)ShapeComp)->InitSphereRadius(Radius);
			((USphereComponent*)ShapeComp)->SetSphereRadius(Radius);
		}
		else
		{
			// Load Mesh dynamically
			if (bUseVisual)
				Mesh = LoadMeshFromPath(FName(*Link->Visual.Mesh));
			else
				Mesh = LoadMeshFromPath(FName(*Link->Collision.Mesh));

			if (!Mesh)
			{
				UE_LOG(LogTemp, Warning, TEXT("Failed to load custom mesh.\n"));
				return false;
			}

		}
	}
	else if (bUseCollision && bUseVisual)
	{
		if (Link->Visual.Mesh.Equals("box", ESearchCase::IgnoreCase))
		{
			Mesh = CubeMesh;
		}
		else if (Link->Visual.Mesh.Equals("cylinder", ESearchCase::IgnoreCase))
		{
			Mesh = CylinderMesh;
		}
		else if (Link->Visual.Mesh.Equals("sphere", ESearchCase::IgnoreCase))
		{
			Mesh = SphereMesh;
		}
		else
		{
			// Load Mesh dynamically
			Mesh = LoadMeshFromPath(FName(*Link->Visual.Mesh));

			if (!Mesh)
			{
				UE_LOG(LogTemp, Warning, TEXT("Failed to load custom mesh.\n"));
				return false;
			}

		}

		if (Link->Collision.Mesh.Equals("box", ESearchCase::IgnoreCase))
		{
			Mesh = CubeMesh;
			ShapeComp = NewObject<UBoxComponent>(Root, FName(Link->Name.GetCharArray().GetData()));
			FVector BoxSize(Scale);
			BoxSize *= 50.f;
			((UBoxComponent*)ShapeComp)->InitBoxExtent(BoxSize);
			((UBoxComponent*)ShapeComp)->SetBoxExtent(BoxSize);
		}
		else if (Link->Collision.Mesh.Equals("cylinder", ESearchCase::IgnoreCase))
		{
			Mesh = CylinderMesh;
			ShapeComp = NewObject<UCapsuleComponent>(Root, FName(Link->Name.GetCharArray().GetData()));
			float Radius = Scale.X * 50.f;
			float Height = Scale.Z * 50.f;
			((UCapsuleComponent*)ShapeComp)->InitCapsuleSize(Radius, Height);
			((UCapsuleComponent*)ShapeComp)->SetCapsuleSize(Radius, Height);
		}
		else if (Link->Collision.Mesh.Equals("sphere", ESearchCase::IgnoreCase))
		{
			Mesh = SphereMesh;
			ShapeComp = NewObject<USphereComponent>(Root, FName(Link->Name.GetCharArray().GetData()));
			float Radius = Scale.X * 50.f;
			((USphereComponent*)ShapeComp)->InitSphereRadius(Radius);
			((USphereComponent*)ShapeComp)->SetSphereRadius(Radius);
		}
		else
		{
			// Load Mesh dynamically
			if (bUseVisual)
				Mesh = LoadMeshFromPath(FName(*Link->Visual.Mesh));
			else
				Mesh = LoadMeshFromPath(FName(*Link->Collision.Mesh));

			if (!Mesh)
			{
				UE_LOG(LogTemp, Warning, TEXT("Failed to load custom mesh.\n"));
				return false;
			}
		}
	}
	else if (!bUseCollision && !bUseVisual) {
		// Create children
		for (int c = 0; c < Node->Children.Num(); c++)
		{
			CreateActorsFromNode(Node->Children[c]);
		}


		return true;
	}

	// Create and configure the Link
	if (ShapeComp)
		MeshComp = NewObject<URStaticMeshComponent>(ShapeComp, FName((Link->Name + "_Visual").GetCharArray().GetData()));
	else
	{
		// Using custom mesh
		MeshComp = NewObject<URStaticMeshComponent>(Root, FName(Link->Name.GetCharArray().GetData()));

	}

	MeshComp->SetStaticMesh(Mesh);
	MeshComp->SetMaterial(0, BasicMaterial);
	MeshComp->SetSimulatePhysics(true);
	MeshComp->RegisterComponent();

	FVector minBounds;
	FVector maxBounds;
	MeshComp->GetLocalBounds(minBounds, maxBounds);

	// Recalculate the numbers with the actual scale. Location value 1 equals maxBounds value in UE4
	FVector LocationCollision = Link->Collision.Location; //All location information should be within the joints tag so LocationCollision and LocationVisual shouldn't be doing anything, should be removed later.
	LocationCollision.Z *= maxBounds.Z * 2;
	LocationCollision.X *= maxBounds.X * 2;
	LocationCollision.Y *= maxBounds.Y * 2;

	FVector LocationVisual = Link->Visual.Location;
	LocationVisual.Z *= maxBounds.Z * 2;
	LocationVisual.X *= maxBounds.X * 2;
	LocationVisual.Y *= maxBounds.Y * 2;

	if (ShapeComp)
	{
		if (Link->Inertial.Mass > 0) {
			ShapeComp->SetMassOverrideInKg(FName("ShapeComp"), Link->Inertial.Mass, true);
		}

		ShapeComp->SetSimulatePhysics(true);
		ShapeComp->SetCollisionObjectType(ECollisionChannel::ECC_PhysicsBody);
		ShapeComp->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
		ShapeComp->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
		ShapeComp->RegisterComponent();

		//Disabling collisions for all shape components turned out to be necessary for the pr2.
		if (!bEnableShapeCollisions) {
			ShapeComp->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
		}
		
		MeshComp->WeldTo(ParentComp);		

		ShapeComp->SetRelativeScale3D(FVector(1, 1, 1));

		ShapeComp->SetWorldLocation(ParentComp->GetComponentLocation());
		ShapeComp->SetWorldRotation(ParentComp->GetComponentRotation());
		ShapeComp->AddLocalOffset(LocationCollision);
		ShapeComp->AddLocalRotation(Link->Collision.Rotation);

		// Weld visual part to the collision body
		MeshComp->SetSimulatePhysics(false);
		MeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		MeshComp->SetCollisionObjectType(ECollisionChannel::ECC_Visibility);
		MeshComp->WeldTo(ShapeComp);
		MeshComp->SetWorldLocation(ParentComp->GetComponentLocation());
		MeshComp->SetWorldRotation(ParentComp->GetComponentRotation());
		MeshComp->AddLocalOffset(LocationVisual);
		MeshComp->AddLocalRotation(Link->Visual.Rotation);
		MeshComp->SetWorldScale3D(Scale);
	}
	else
	{
		if (Link->Inertial.Mass > 0) {
			MeshComp->SetMassOverrideInKg(NAME_None, Link->Inertial.Mass, true);
		}

		

		MeshComp->SetCollisionObjectType(ECollisionChannel::ECC_GameTraceChannel1);
		

		for (FString linkName : collisionFilterArr) {
			if (Link->Name.Contains(linkName)) {
				MeshComp->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);
				MeshComp->WeldTo(ParentComp);
				break;
			}
		}

		//if (Link->Name.Contains("wheel_link") || Link->Name.Contains("shoulder") || Link->Name.Contains("arm") || Link->Name.Contains("finger_link")) {
		//	//Prevents certain links from colliding with themselves.
		//	MeshComp->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Ignore);
		//	MeshComp->WeldTo(ParentComp);
		//}
		
		MeshComp->SetWorldLocation(ParentComp->GetComponentLocation());
        UE_LOG(LogTemp, Warning, TEXT("Add MeshComp [%s], Parent [%s], Parent WorldRotation = [%s]"), *MeshComp->GetName(), *ParentComp->GetName(), *ParentComp->GetComponentRotation().ToString()); 
		MeshComp->SetWorldRotation(ParentComp->GetComponentRotation());
		MeshComp->AddRelativeLocation(LocationVisual);
		MeshComp->SetRelativeRotation(Link->Visual.Rotation);
		
	}

	// Create and configure the Joint if this is not the topmost link
	if (ParentLink)
	{
		UPhysicsConstraintComponent* Constraint = NewObject<UPhysicsConstraintComponent>(ParentComp, FName(Joint->Name.GetCharArray().GetData()));
		FConstraintInstance ConstraintInstance = SetConstraint(Joint);
		ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = false;
		ConstraintInstance.ProfileInstance.ConeLimit.bSoftConstraint = false;
	
								
		FRotator rotTarget(0.f, 0.f, 0.f);				
		if (Joint->Name.Contains("r_shoulder_lift")) { //Example of how certain joints can be given a rotation target
			rotTarget= FRotator(-30.f, 0.f, 0.f);
		}	
		
		
		if (bEnableAngularMotors) { //Example of how the angular motors can be enabled
			// Damping requires velocity drive to be enabled
			ConstraintInstance.ProfileInstance.AngularDrive.AngularVelocityTarget = FVector(0.f, 0.f, 0.f);
			ConstraintInstance.ProfileInstance.AngularDrive.SlerpDrive.bEnableVelocityDrive = true;
			ConstraintInstance.ProfileInstance.AngularDrive.SwingDrive.bEnableVelocityDrive = true;
			ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive.bEnableVelocityDrive = true;

			ConstraintInstance.ProfileInstance.AngularDrive.AngularDriveMode = EAngularDriveMode::TwistAndSwing;
			ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
			ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
			ConstraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);

			ConstraintInstance.ProfileInstance.AngularDrive.OrientationTarget = rotTarget;

			ConstraintInstance.ProfileInstance.AngularDrive.SlerpDrive.bEnablePositionDrive = true;
			ConstraintInstance.ProfileInstance.AngularDrive.SwingDrive.bEnablePositionDrive = true;
			ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive.bEnablePositionDrive = true;

			ConstraintInstance.ProfileInstance.AngularDrive.SlerpDrive.Damping = Link->Inertial.Mass * 50000;
			ConstraintInstance.ProfileInstance.AngularDrive.SlerpDrive.Stiffness = Link->Inertial.Mass * 120000;

			ConstraintInstance.ProfileInstance.AngularDrive.SwingDrive.Damping = Link->Inertial.Mass * 50000;
			ConstraintInstance.ProfileInstance.AngularDrive.SwingDrive.Stiffness = Link->Inertial.Mass * 120000;

			ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive.Damping = Link->Inertial.Mass * 50000;
			ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive.Stiffness = Link->Inertial.Mass * 120000; 
		}

		//--
		Constraint->ConstraintInstance = ConstraintInstance;		
		Constraint->SetDisableCollision(true);
		Constraint->AttachToComponent(ParentComp, FAttachmentTransformRules(EAttachmentRule::KeepRelative, false));
		Constraint->RegisterComponent();
		Constraint->ApplyWorldOffset(FVector(0), false);

		FVector LocationJoint = Joint->Location;
		//LocationJoint contains the location values as seen in the URDF, must be scaled by 100 when importing to UE4
		LocationJoint.Z = (LocationJoint.Z * 100); 
		LocationJoint.X = (LocationJoint.X * 100);
		LocationJoint.Y = (LocationJoint.Y * 100);


		if (ShapeComp)
		{
			ShapeComp->AddRelativeLocation(LocationJoint);
			ShapeComp->AddRelativeRotation(Joint->Rotation);
			LinkOriginLocation = ShapeComp->GetRelativeTransform().GetLocation();
		}
		else
		{						
			MeshComp->AddRelativeLocation(LocationJoint);
			MeshComp->AddRelativeRotation(Joint->Rotation);
			LinkOriginLocation = MeshComp->GetRelativeTransform().GetLocation();
		}

		// Prismatic joints need special positioning due to the style of constraint limits
		if (Joint->Type.Equals("prismatic", ESearchCase::IgnoreCase))
		{
			FVector MovementVector(Joint->Axis * (Joint->LowerLimit + Joint->UpperLimit));
			OriginLocations.Add(Joint->Child, MovementVector);
			if (ShapeComp)
			{
				ShapeComp->AddRelativeLocation(MovementVector);
			}
			else
			{
				MeshComp->AddRelativeLocation(MovementVector);
			}
		}

		Constraint->SetRelativeRotation(Joint->Rotation);

		FConstrainComponentPropName ConstrainCompName;

		Constraint->ConstraintActor1 = this;

		Constraint->ConstraintActor2 = this;		


		if (ShapeComp)
		{
			Constraint->SetWorldLocation(ShapeComp->GetComponentLocation());
			Constraint->SetConstrainedComponents(ParentLink, NAME_None, ShapeComp, NAME_None);
		}
		else
		{
			Constraint->SetWorldLocation(MeshComp->GetComponentLocation());
			Constraint->SetConstrainedComponents(ParentLink, NAME_None, MeshComp, NAME_None);
            /* UE_LOG(LogTemp, Log, TEXT("[%s] [%s] [%x] [%x]"),
                *Constraint->ComponentName1.ComponentName.ToString(),
                *Constraint->ComponentName2.ComponentName.ToString(),
                Constraint->OverrideComponent1.Get(),
                Constraint->OverrideComponent2.Get()); */
		}

        FRotator ParentRotation = ParentLink->GetComponentRotation();
        FRotator ChildRotation = ShapeComp ? (ShapeComp->GetComponentRotation()) : (MeshComp->GetComponentRotation());
        FQuat InitialRotationRel = FQuat(ParentRotation).Inverse() * FQuat(ChildRotation);

        OriginRotations.Add(Joint->Name, InitialRotationRel); 
    	JointComponents.Add(Joint->Name, Constraint);
	}

	if (ShapeComp)
		LinkComponents.Add(Link->Name, ShapeComp);
	else
		LinkComponents.Add(Link->Name, MeshComp);	

	// Create children
	for (int c = 0; c < Node->Children.Num(); c++)
	{
		CreateActorsFromNode(Node->Children[c]);
	}


	return true;
}

FConstraintInstance ARRobot::NewConstraintInstanceFixed()
{
	// Set the constraint limits
	FConstraintInstance ConstraintInstance;
	ConstraintInstance.SetDisableCollision(true);
	ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
	ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
	ConstraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
	ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
	ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
	ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0);
	ConstraintInstance.AngularRotationOffset = FRotator(0, 0, 0);

	return ConstraintInstance;
}

//TEMPLATE Load Obj From Path
template <typename ObjClass>
FORCEINLINE ObjClass* ARRobot::LoadObjFromPath(const FName& Path)
{
	if (Path == NAME_None) return NULL;

	return Cast<ObjClass>(StaticLoadObject(ObjClass::StaticClass(), NULL, *Path.ToString()));
}

// Load Static Mesh From Path 
FORCEINLINE UStaticMesh* ARRobot::LoadMeshFromPath(const FName& Path)
{
	if (Path == NAME_None) return NULL;

	return LoadObjFromPath<UStaticMesh>(Path);
}

void ARRobot::ParseURDF()
{
	// Callback Object for parser
	FRURDFParser Call(this);

	// Error and description Message
	FText Error = NSLOCTEXT("RFastXmlCallbackRobotError", "FFastXmlError", "Error in the execution of the XML-Parser");

	// Default Line for OutErrorLineNumber
	int32 LineNumb = -1;

	// indicates if the robot could be created or not
	bool Success;

	// Remove linebreaks and tabs
	//XmlUrdf = XmlUrdf.Trim().TrimTrailing();
	XmlUrdf.TrimStartAndEndInline();
	XmlUrdf = XmlUrdf.Replace(L"\n", L" ");
	XmlUrdf = XmlUrdf.Replace(L"\r", L"");
	XmlUrdf = XmlUrdf.Replace(L"\t", L" ");

	if (XmlUrdf.IsEmpty()) return;

	// Create a copy of the URDF since the parser modifies the input string
	FString StringToParse = XmlUrdf;

	Success = FFastXml::ParseXmlFile((IFastXmlCallback*)&Call, TEXT(""), StringToParse.GetCharArray().GetData(), nullptr, false, false, Error, LineNumb);

	CreateRobot();

	if (!Success)
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to create Robot\n"));
	}
}

// Sets the constraints according to the urdf
FConstraintInstance ARRobot::SetConstraint(FRJoint* Joint) 
{
	// Create a basic fixed constraint
	FConstraintInstance ConstraintInstance = NewConstraintInstanceFixed();
	ELinearConstraintMotion LinearConstraintMotion = ELinearConstraintMotion::LCM_Free;
	EAngularConstraintMotion AngularConstraintMotion = EAngularConstraintMotion::ACM_Free;
	ConstraintInstance.ProfileInstance.AngularDrive.AngularDriveMode = EAngularDriveMode::TwistAndSwing;

	if (Joint->Type.Equals("fixed", ESearchCase::IgnoreCase))
		return ConstraintInstance;

	if (Joint->Type.Equals("floating", ESearchCase::IgnoreCase))
	{
		ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.f);
		ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.f);
		ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.f);
		return ConstraintInstance;
	}

	//Currently simplified limit (lower + upper as a value).
	//lower, upper A(radians for revolute joints, meters for prismatic joints)
	float SimpleLimit = 0.f;
	if (Joint->Type.Equals("prismatic", ESearchCase::IgnoreCase) || Joint->Type.Equals("revolute", ESearchCase::IgnoreCase)) {
		SimpleLimit = FMath::Abs(Joint->LowerLimit) + FMath::Abs(Joint->UpperLimit);
		LinearConstraintMotion = ELinearConstraintMotion::LCM_Limited;
		AngularConstraintMotion = EAngularConstraintMotion::ACM_Limited;


	}

	// Set the movable axes
	if (Joint->Axis.X == 1) {
		if (Joint->Type.Equals("prismatic", ESearchCase::IgnoreCase))
		{
			ConstraintInstance.SetLinearXLimit(LinearConstraintMotion, SimpleLimit);
			ConstraintInstance.ProfileInstance.LinearDrive.XDrive.bEnablePositionDrive = true;
			ConstraintInstance.ProfileInstance.LinearDrive.XDrive.MaxForce = Joint->Effort;
		}
		else
		{
			// Angular motor on X axis needs SLERP drive mode
			ConstraintInstance.SetAngularTwistLimit(AngularConstraintMotion, SimpleLimit);
			ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Limited, 0.1f);
			ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 0.1f);
			ConstraintInstance.ProfileInstance.AngularDrive.AngularDriveMode = EAngularDriveMode::SLERP;
			ConstraintInstance.ProfileInstance.AngularDrive.SlerpDrive.MaxForce = Joint->Effort;
			ConstraintInstance.ProfileInstance.AngularDrive.SlerpDrive.bEnablePositionDrive = true;
		}
	}
	if (Joint->Axis.Y == 1) {
		if (Joint->Type.Equals("prismatic", ESearchCase::IgnoreCase))
		{
			ConstraintInstance.SetLinearYLimit(LinearConstraintMotion, SimpleLimit);
			ConstraintInstance.ProfileInstance.LinearDrive.YDrive.bEnablePositionDrive = true;
			ConstraintInstance.ProfileInstance.LinearDrive.YDrive.MaxForce = Joint->Effort;
		}
		else
		{
			ConstraintInstance.SetAngularSwing2Limit(AngularConstraintMotion, SimpleLimit);
			ConstraintInstance.ProfileInstance.AngularDrive.SlerpDrive.MaxForce = Joint->Effort;
			ConstraintInstance.ProfileInstance.AngularDrive.SwingDrive.bEnablePositionDrive = true;
		}

	}
	if (Joint->Axis.Z == 1) {
		if (Joint->Type.Equals("prismatic", ESearchCase::IgnoreCase))
		{
			ConstraintInstance.SetLinearZLimit(LinearConstraintMotion, SimpleLimit);
			ConstraintInstance.ProfileInstance.LinearDrive.ZDrive.bEnablePositionDrive = true;
			ConstraintInstance.ProfileInstance.LinearDrive.ZDrive.MaxForce = Joint->Effort;
		}
		else
		{
			ConstraintInstance.SetAngularSwing1Limit(AngularConstraintMotion, SimpleLimit);
			ConstraintInstance.ProfileInstance.AngularDrive.SlerpDrive.MaxForce = Joint->Effort;
			ConstraintInstance.ProfileInstance.AngularDrive.SwingDrive.bEnablePositionDrive = true;
		}
	}
	
	ConstraintInstance.AngularRotationOffset = FRotator(0, 0, 0);

	if (Joint->Type.Equals("planar")) {
		// A Constraint for planar type. This joint allows motion in a plane perpendicular to the axis. 

		if (Joint->Axis.Z == 1) {
			ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0);
			ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0);
			ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0);
		}
		else if (Joint->Axis.X == 1) {
			ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
			ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0);
			ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0);
		}
		else if (Joint->Axis.Y == 1) {
			ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0);
			ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
			ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0);
		}

	}

	return ConstraintInstance;
}

void ARRobot::BuildTree(FRNode* Node)
{
	if (!Node)
	{
		return;
	}
	// Go through all joints to find the ones that use the current Node as parent
	for (int c = 0; c < Joints.Num(); c++)
	{
		bool bHasFoundMatch = false;
		if (Joints[c].Parent.Equals(Node->Link.Name))
		{
			// Find the child link which parent is the current node
			for (int i = 0; i < Links.Num(); i++)
			{
				if (Links[i].Name.Equals(Joints[c].Child))
				{
					FRNode* ChildNode = new FRNode();
					ChildNode->Parent = Node;
					ChildNode->Joint = Joints[c];
					ChildNode->Link = Links[i];
					Node->Children.Add(ChildNode);

					Joints.RemoveAt(c);
					Links.RemoveAt(i);
					bHasFoundMatch = true;
					break;
				}
			}
		}
		// Decrement the joint counter as the array automatically shrinks on removal
		if (bHasFoundMatch) c--;
	}

	for (int c = 0; c < Node->Children.Num(); c++)
	{
		BuildTree(Node->Children[c]);
	}
}


bool ARRobot::RotateJoint(FString Name, FRotator TargetRotation)
{
	UE_LOG(LogTemp, Display, TEXT("RotateJoint %s"), *Name);
	TArray<UActorComponent*> Children = GetComponents().Array();
	for (int c = 0; c < Children.Num(); c++)
	{
		UActorComponent* SceneComp = Children[c];
		if (SceneComp->GetName().Equals(Name, ESearchCase::IgnoreCase))
		{
			UPhysicsConstraintComponent* Constraint = (UPhysicsConstraintComponent*)SceneComp;

			FConstraintInstance* ConstraintInstance = &(Constraint->ConstraintInstance);
			Constraint->SetAngularOrientationTarget(TargetRotation);
			return true;
		}

	}


	return false;
}

bool ARRobot::MovePrismaticJoint(FString Name, FVector TargetPosition)
{
	UE_LOG(LogTemp, Display, TEXT("MovePrismaticJoint %s"), *Name);
	TArray<UActorComponent*> Children = GetComponents().Array();
	for (int c = 0; c < Children.Num(); c++)
	{
		UActorComponent* ActorComp = Children[c];
		if (ActorComp->GetName().Equals(Name, ESearchCase::IgnoreCase))
		{
			UPhysicsConstraintComponent* Constraint = (UPhysicsConstraintComponent*)ActorComp;

			FConstraintInstance* ConstraintInstance = &(Constraint->ConstraintInstance);
			ConstraintInstance->SetLinearPositionTarget(TargetPosition);
			
			return true;
		}

	}

	return true;
}

// Get Joint Position in *Degrees*
float ARRobot::GetJointPosition(FString JointName)
{
    UPhysicsConstraintComponent* Joint = JointComponents[JointName];

    FString ParentCompName = Joint->ComponentName1.ComponentName.ToString();
    FString ChildCompName = Joint->ComponentName2.ComponentName.ToString();
    UPrimitiveComponent* ParentComponent = LinkComponents[ParentCompName];
    UPrimitiveComponent* ChildComponent = LinkComponents[ChildCompName];
    if (ParentComponent && ChildComponent)
    {
        FRotator ParentRotation = ParentComponent->GetComponentRotation();
        FRotator ChildRotation = ChildComponent->GetComponentRotation();
        FQuat CurrentRotationRel = FQuat(ParentRotation).Inverse() * FQuat(ChildRotation);
        FQuat InitialRotationRel = OriginRotations[Joint->GetName()];
        FQuat QRel = CurrentRotationRel * InitialRotationRel.Inverse();
        FVector Axis; float Angle;
        QRel.ToAxisAndAngle(Axis, Angle);

        // Get Axis
        auto MotionSwing1 = Joint->ConstraintInstance.GetAngularSwing1Motion();
        auto MotionSwing2 = Joint->ConstraintInstance.GetAngularSwing2Motion();
        auto MotionTwist = Joint->ConstraintInstance.GetAngularTwistMotion();
        bool rotationX = false, rotationY = false, rotationZ = false;
        if (MotionSwing1 == EAngularConstraintMotion::ACM_Free || MotionSwing1 == EAngularConstraintMotion::ACM_Limited)
            rotationZ = true;
        if (MotionSwing2 == EAngularConstraintMotion::ACM_Free || MotionSwing2 == EAngularConstraintMotion::ACM_Limited)
            rotationY = true;
        if (MotionTwist == EAngularConstraintMotion::ACM_Free || MotionTwist == EAngularConstraintMotion::ACM_Limited)
            rotationX = true;

        if ((!rotationX && !rotationY && !rotationZ) || (rotationX && rotationY) || (rotationX && rotationZ) || (rotationY && rotationZ))
        {
            // not a hinged joint
            // UE_LOG(LogTemp, Error, TEXT("Joint [%s] is not a hinged joint with DOF=1"), *Joint->GetName());
            return 0;
        }
        else
        {
            // a hinged joint
            FVector RefAxis;
            if (rotationX) RefAxis = FVector(1, 0, 0);
            if (rotationY) RefAxis = FVector(0, 1, 0);
            if (rotationZ) RefAxis = FVector(0, 0, 1);

            float ResultAngle = FVector::DotProduct(Axis.GetSafeNormal(), RefAxis) * FMath::RadiansToDegrees(Angle);
            while (ResultAngle > 180)
                ResultAngle -= 360;
            while (ResultAngle < -180 )
                ResultAngle += 360;



            return ResultAngle;
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Joint [%s] doesn't have parent or child."), *Joint->GetName());
        return 0.0;
    }
}

// Get Joint Velocity in Deg/s
float ARRobot::GetJointVelocity(FString JointName)
{
    UPhysicsConstraintComponent* Joint = JointComponents[JointName];

    FString ParentCompName = Joint->ComponentName1.ComponentName.ToString();
    FString ChildCompName = Joint->ComponentName2.ComponentName.ToString();
    UPrimitiveComponent* ParentComponent = LinkComponents[ParentCompName];
    UPrimitiveComponent* ChildComponent = LinkComponents[ChildCompName];

    if (ParentComponent && ChildComponent)
    {
        // Get Rotation Axis
        auto MotionSwing1 = Joint->ConstraintInstance.GetAngularSwing1Motion();
        auto MotionSwing2 = Joint->ConstraintInstance.GetAngularSwing2Motion();
        auto MotionTwist = Joint->ConstraintInstance.GetAngularTwistMotion();
        bool rotationX = false, rotationY = false, rotationZ = false;
        if (MotionSwing1 == EAngularConstraintMotion::ACM_Free || MotionSwing1 == EAngularConstraintMotion::ACM_Limited)
            rotationZ = true;
        if (MotionSwing2 == EAngularConstraintMotion::ACM_Free || MotionSwing2 == EAngularConstraintMotion::ACM_Limited)
            rotationY = true;
        if (MotionTwist == EAngularConstraintMotion::ACM_Free || MotionTwist == EAngularConstraintMotion::ACM_Limited)
            rotationX = true;

        if ((!rotationX && !rotationY && !rotationZ) || (rotationX && rotationY) || (rotationX && rotationZ) || (rotationY && rotationZ))
        {
            // not a hinged joint
            UE_LOG(LogTemp, Error, TEXT("Joint [%s] is not a hinged joint with DOF=1"), *Joint->GetName());
            return 0.0;
        }
        else
        {
            // a hinged joint
            FVector RefAxis;
            if (rotationX) RefAxis = FVector(1, 0, 0);
            if (rotationY) RefAxis = FVector(0, 1, 0);
            if (rotationZ) RefAxis = FVector(0, 0, 1);

            // Get Axis ? 
            FQuat JointQuat = Joint->GetComponentTransform().GetRotation();
            FVector GlobalAxis = JointQuat.RotateVector(RefAxis); // Rotation Axis in Global Frame

            FVector ParentAvel = ParentComponent->GetPhysicsAngularVelocityInRadians();
            FVector ChildAvel = ChildComponent->GetPhysicsAngularVelocityInRadians();
            float HingeVel = FVector::DotProduct(ChildAvel - ParentAvel, GlobalAxis); 

            /* UE_LOG(LogTemp, Warning, TEXT("Joint [%s]: HingeVel = %.3f; ChildAvel [%s], ParentAvel [%s], Axis [%s] (global)"),
                *Joint->GetName(), HingeVel, *ChildAvel.ToString(), *ParentAvel.ToString(), *GlobalAxis.ToString()); */

            return HingeVel;
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Joint [%s] doesn't have parent or child."), *Joint->GetName());
        return 0.0;
    }
}

void ARRobot::AddForceToJoint(FString JointName, float Force)
{
    if (!JointComponents.Find(JointName))
    {
        UE_LOG(LogTemp, Error, TEXT("Unable to find Joint [%s]!"), *JointName);
        return;
    }

    UPhysicsConstraintComponent* Joint = JointComponents[JointName];

    FString ParentCompName = Joint->ComponentName1.ComponentName.ToString();
    FString ChildCompName = Joint->ComponentName2.ComponentName.ToString();
    UPrimitiveComponent* ParentComponent = LinkComponents[ParentCompName];
    UPrimitiveComponent* ChildComponent = LinkComponents[ChildCompName];

    if (ParentComponent && ChildComponent)
    {
        // Get Rotation Axis
        auto MotionSwing1 = Joint->ConstraintInstance.GetAngularSwing1Motion();
        auto MotionSwing2 = Joint->ConstraintInstance.GetAngularSwing2Motion();
        auto MotionTwist = Joint->ConstraintInstance.GetAngularTwistMotion();
        bool rotationX = false, rotationY = false, rotationZ = false;
        if (MotionSwing1 == EAngularConstraintMotion::ACM_Free || MotionSwing1 == EAngularConstraintMotion::ACM_Limited)
            rotationZ = true;
        if (MotionSwing2 == EAngularConstraintMotion::ACM_Free || MotionSwing2 == EAngularConstraintMotion::ACM_Limited)
            rotationY = true;
        if (MotionTwist == EAngularConstraintMotion::ACM_Free || MotionTwist == EAngularConstraintMotion::ACM_Limited)
            rotationX = true;

        if ((!rotationX && !rotationY && !rotationZ) || (rotationX && rotationY) || (rotationX && rotationZ) || (rotationY && rotationZ))
        {
            // not a hinged joint
            // UE_LOG(LogTemp, Error, TEXT("Joint [%s] is not a hinged joint with DOF=1"), *Joint->GetName());
        }
        else
        {
            // a hinged joint
            FVector RefAxis;
            if (rotationX) RefAxis = FVector(1, 0, 0);
            if (rotationY) RefAxis = FVector(0, 1, 0);
            if (rotationZ) RefAxis = FVector(0, 0, 1);

            // Get Axis ?
            FQuat JointQuat = Joint->GetComponentTransform().GetRotation();
            FVector GlobalAxis = JointQuat.RotateVector(RefAxis); // Rotation Axis in Global Frame

            FVector TorqueToAdd = GlobalAxis * Force;
            ChildComponent->AddTorqueInRadians(TorqueToAdd);
            ParentComponent->AddTorqueInRadians(-TorqueToAdd);
        }
    }
}
