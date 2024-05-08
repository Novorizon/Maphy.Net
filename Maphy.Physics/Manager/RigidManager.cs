
using Maphy.Mathematics;
using System.Collections.Generic;

namespace Maphy.Physics
{
    public class RigidManager
    {
        public static ulong id = 1000000;
        private static readonly Dictionary<ulong, Rigid> datas = new Dictionary<ulong, Rigid>();
        public static ulong Generateid()
        {
            return id++;
        }


        public static Rigid CreateRigid()
        {
            Rigid rigid = new Rigid(id);
            datas.Add(id, rigid);
            id++;
            return rigid;
        }

        //public static Rigid CreateRigid(fix3 position, quaternion rotation)
        //{
        //    Rigid rigid = new Rigid(id++, position, rotation);
        //    return rigid;
        //}

        public static Rigid GetData(ulong id)
        {
            if (datas.ContainsKey(id))
            {
                return datas[id];
            }

            return Rigid.Default;
        }

        public static void SetData(Rigid rigid)
        {
            if (datas.ContainsKey(rigid.id))
            {
                datas[rigid.id] = rigid;
            }
            else
            {
                datas.Add(rigid.id, rigid);
            }
        }


        public static void Remove(ulong id)
        {
            if (datas.ContainsKey(id))
            {
                datas.Remove(id);
            }
        }


        public static void Remove(Rigid a)
        {
            if (datas.ContainsKey(a.id))
            {
                datas.Remove(a.id);
            }
        }


        //public static Rigid AddRigid(fix3 position, quaternion rotation)
        //{
        //    Rigid rigid = new Rigid(id++, position, rotation);
        //    return rigid;
        //}
    }
}