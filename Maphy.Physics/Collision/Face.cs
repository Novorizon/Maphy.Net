using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Maphy.Physics
{
    public enum Face 
{
        None,
        Left=1,
        Right=1<<1,
        Top=1<<2,
        Bottom=1<<3,
        Front=1<<4,
        Back=1<<5,
}
}
