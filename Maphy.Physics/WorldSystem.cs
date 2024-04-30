
//using Maphy.Mathematics;
//using Unity.Entities;

//namespace Maphy.Physics
//{
//    public class WorldComponent:IComponentData
//    {
//        public void Update()
//        {

//        }
//    }

//    public class WorldSystem : SystemBase
//    {
//            protected override void OnUpdate()
//            {
//                var deltaTime = math.min(0.05f, SystemAPI.Time.DeltaTime);

//                Entities.ForEach((ref WorldComponent world) =>
//                {
//                    world.Update();
//                }).ScheduleParallel();

//            }
//        }
//    }
