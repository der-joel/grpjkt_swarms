# 3D Unity Swarm Simultion
A swarm simulation algorithm for Unity Engine.
Based on Craig Reynolds [boids-Algorithm](https://www.red3d.com/cwr/boids/).

## Steering Patterns
There is no shared memory between multiple agents. The swarm movement is created through aggregation of the individual behaviour.
The individual agents act based on a few movement patterns defined by Craig Reynolds in [Steering Behaviors For Autonomous Characters](https://www.red3d.com/cwr/papers/1999/gdc99steer.html).

This implementation supports the following steering patterns:
- Seek
- Arrival
- Wander
- Cohesion
- Separation
- Alignment
- Obstacle Avoidance (using a different concept)
- Hide
- Flee

## Intergration in existing projects
To integrate this into your project simply place the [agent-script](Assets/Scripts/Agent.cs) and 
the [behaviour-script](Assets/Scripts/SteeringBehaviours.cs) somewhere inside your project (in the same folder).

If you wish for some demonstration you can put the whole intergrate the whole [asset-folder](Assets)) into your project assets.
Inside you will find a [sample-scene](Assets/Scenes/SampleScene.unity) for demonstration purposes and some textures.
If you need an example for possible swarm agents, take a look at the [prefab-folder](Assets/Prefabs).

## Usage
Any `gameObject` can be turned into a swarm agent.
The only requirement is that the following components are attached to the parent-object:
- the [agent-script](Assets/Scripts/Agent.cs)
- a `Rigidbody`
- one or multiple `Collider`'s of any size or form with `isTrigger = true` (used to detect nearby enemies)

You can customize the behaviour of any individual agent by adjusting values in the inpector.

The **Ratio**-Tab lets you configure how the different steering behaviours are mixed to create the agents movement:
>Example: for agents to only use the "Wander" movement pattern, set **wanderFactor** to 1 and all other factors to 0.

For fine-tuning the different movement patterns can also be customized:
>Example: use **Arrival Slowing Distance** to customize the distance to the target, at which the agent starts slowing down.

## Demo-Video
[Click here](https://streamable.com/8y2kxe) to watch a demo video.

## Used Packages and Sources
- [Stylised Submarine](https://assetstore.unity.com/packages/3d/vehicles/stylized-submarine-143306) (only character and animations were used) 
- [Sardine](https://assetstore.unity.com/packages/3d/characters/animals/fish/sardine-37963) (only character and animations were used) 
