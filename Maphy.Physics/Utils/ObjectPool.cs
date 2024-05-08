using System.Collections.Generic;

public class ObjectPool<T> where T : notnull
{
    private T value;
    private Queue<T> pool = new Queue<T>();

    public ObjectPool(T value, int size)
    {
        this.value = value;
        InitializePool(size);
    }

    private void InitializePool(int size)
    {
        for (int i = 0; i < size; i++)
        {
            T obj = GameObject.Instantiate(value);
            obj.gameObject.SetActive(false);
            pool.Enqueue(obj);
        }
    }

    public T GetObject()
    {
        if (pool.Count > 0)
        {
            T obj = pool.Dequeue();
            obj.gameObject.SetActive(true);
            return obj;
        }
        else
        {
            T newObj = GameObject.Instantiate(prefab);
            newObj.gameObject.SetActive(true);
            return newObj;
        }
    }

    public void ReturnObject(T obj)
    {
        obj.gameObject.SetActive(false);
        pool.Enqueue(obj);
    }
}
