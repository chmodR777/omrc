#pragma once
// #pragma warning(disable:4996)
#include <vector>
#include <map>
namespace OMDB
{
	template<typename K,typename V>
	class FastTable
	{
	public:
		bool insert(K key,V value);
		bool exist(K key);
		bool erase(K key);
		void clear();
		V& operator[](K key);
		std::vector<V>& values();
		~FastTable();
	private:
		std::vector<V>		     data;
		std::map<K, size_t> indexes;
	};

	template<typename K, typename V>
	OMDB::FastTable<K, V>::~FastTable()
	{
		data.clear();
		indexes.clear();
	}

	template<typename K, typename V>
	V& FastTable<K, V>::operator[](K key)
	{
		size_t index = indexes[key];
		return data[index];
	}

	template<typename K, typename V>
	std::vector<V>& OMDB::FastTable<K, V>::values()
	{
		return data;
	}

	template<typename K, typename V>
	bool FastTable<K, V>::exist(K key)
	{
		return indexes.find(key) != indexes.end();
	}

	template<typename K, typename V>
	bool FastTable<K, V>::erase(K key)
	{
		auto indexIter = indexes.find(key);
		if (indexIter == indexes.end())
			return false;

		size_t index = indexes[key];
		indexes.erase(indexIter);

		auto value = data[index];
		auto valueIter = std::find(data.begin(), data.end(), value);
		if (valueIter != data.end())
			data.erase(valueIter);
		return true;
	}

	template<typename K, typename V>
	bool FastTable<K, V>::insert(K key, V value)
	{
		if (exist(key))
			return false;
		indexes.emplace(key, data.size());
		data.push_back(value);
		
		return true;
	}

	template<typename K, typename V>
	void FastTable<K, V>::clear()
	{
		data.clear();
		indexes.clear();
	}
}


