
using Maphy.Mathematics;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class ShapeManager 
    {
        public static ulong id = 1000000;
        private static readonly Dictionary<ulong, Shape> datas = new Dictionary<ulong, Shape>();
        private static readonly Dictionary<ulong, Shape> aabbs = new Dictionary<ulong, Shape>();
        private static readonly Dictionary<ulong, Shape> obbs = new Dictionary<ulong, Shape>();
        private static readonly Dictionary<ulong, Shape> spheres = new Dictionary<ulong, Shape>();
        private static readonly Dictionary<ulong, Shape> capsules = new Dictionary<ulong, Shape>();
        public static ulong GenerateId()
        {
            return id++;
        }


        public static AABB CreateAABB(fix3 center, fix3 size)
        {
            ulong id = GenerateId();
            AABB aabb = new AABB(center, size, id);
            datas.Add(id, aabb);
            return aabb;
        }

        public static Shape GetData(ulong id)
        {
            if (datas.ContainsKey(id))
            {
                return datas[id];
            }

            return AABB.Default;
        }

        public static void SetData(Shape a)
        {
            if (datas.ContainsKey(a.Id))
            {
                datas[id]=a;
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


        public static void Remove(Shape a)
        {
            if (datas.ContainsKey(a.Id))
            {
                datas.Remove(a.Id);
            }
        }
    }
}