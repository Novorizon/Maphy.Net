
using Maphy.Mathematics;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class AabbManager :ShapeManager
    {
        private static readonly Dictionary<ulong ,AABB> datas=new Dictionary<ulong, AABB> ();


        public static AABB Create(fix3 center, fix3 size)
        {
            ulong id = GenerateId();
            AABB aabb = new AABB(center, size, id);
            datas.Add(id, aabb);
            return aabb;
        }
        public static AABB GetData(ulong id)
        {
            if(datas.ContainsKey(id))
            {
                return datas[id];
            }

            return AABB.Default;
        }

        public static void SetData(AABB a)
        {
            if (datas.ContainsKey(a.Id))
            {
                datas[id].Copy(a);
            }
            else
            {
                datas[a.Id] = a;
            }
        }


        public static void Remove(ulong id)
        {
            if (datas.ContainsKey(id))
            {
                datas.Remove(id);
            }
        }


        public static void Remove(AABB a)
        {
            if (datas.ContainsKey(a.Id))
            {
                datas.Remove(a.Id);
            }
        }
    }
}