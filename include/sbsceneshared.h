#pragma once

#include <sbtypes.h>
#include <sbtree.h>
#include <sbscene.h>
#include <sbshared.h>

#include <map>

/** SceneShared
	共有メモリ空間に置かれるシーン
 **/

namespace Scenebuilder{;

/** 共有メモリ上のシーンレイアウト
	[基本情報セクション]
	- 各セクションの要素数
	- 各セクションのエントリのバイト数
	- 各セクションの先頭アドレス

	[オブジェクトセクション]
	- [type, parID, numChildren]

	[親子関係セクション]
	- [parID, childID]

	[リンク関係セクション]
	- [srcID, destID]

	[プロパティセクション]
	- オブジェクト種別のプロパティの配列

 **/

class SceneShared : public SceneBase{
public:
	/** コンフィグ情報
		- 各項目の要素数を指定する
		- Create関数に渡す
	 **/
	struct Config{
		/// default maximum number of elements
		enum{
			defObjectMax   = 1000,
			defChildrenMax = 1000,
			defLinkMax     = 1000,
			defLocMax      = 1000,
			defPropMax     = 1000,
		};

		/// maximum number of elements
		size_t				maxObjects;
		size_t				maxChildren;
		size_t				maxLinks;
		size_t				maxLocs;
		map<int, size_t>	maxProps;

	public:
		/// maximum number of properties. returns defMax if not specified
		size_t	MaxProps(int typeId) const{
			map<int, size_t>::const_iterator it = maxProps.find(typeId);
			return (it == maxProps.end() ? defPropMax : it->second);
		}
		
		Config(){
			maxObjects  = defObjectMax;
			maxChildren = defChildrenMax;
			maxLinks    = defLinkMax;
			maxLocs     = defLocMax;
		}
	};

private:
	struct Header{
		/// header 'S''B'
		char		code[2];
		/// number of object types
		size_t		numObjectTypes;
		/// maximum object type id
		int			maxObjectTypeId;
		/// total number of bytes
		size_t		szTotal;
		/// init
		void Init(TypeDB* db, const Config& conf);
	};

	struct Object{
		bool	    alive;			///< alive or not
		bool	    original;		///< orignal or clone
		int		    parId;			///< parent object id
		int		    numChildren;	///< number of child objects
		int		    numLinks;		///< number of links
		int		    childFirst;		///< list node index of first child
		int		    childLast;		///< list node index of last child
		int		    linkFirst;		///< list node index of first link
		int		    linkLast;		///< list node index of last link
		int		    type;			///< object type
		str256_t	name;			///< object name
		int		    propId;			///< property id
		int		    locId;			///< location id

		void Init();
	};

	struct Location{
		int		    id;
		str256_t	path;
	};

	struct Link{
		int			id;
		str256_t	name;
	};
	struct FindById{
		int			id;
		bool		operator()(int _id){
			return _id == id;
		}
		FindById(int _id):id(_id){}
	};
	struct FindLinkById{
		int	 id;
		bool operator()(const Link& link){
			return link.id == id;
		}
		FindLinkById(int _id):id(_id){}
	};
	struct FindLinkByName{
		const char* name;
		bool operator()(const Link& link){
			return link.name == name;
		}
		FindLinkByName(const char* _name):name(_name){}
	};

	template<class T>
	struct ListNode{
		bool	alive;
		int		prev;			///< index of previous list node
		int		next;			///< index of next list node
		T		val;
	};

	struct ArrayHeader{
		size_t		nmax;		///< maximum number of elements
		size_t		sz;			///< size of one element
		size_t		num;		///< number of elements
	};

	template<typename T>
	struct Array{
		ArrayHeader*	header;
		byte*			buf;
		
		// init
		void Init(byte*& ptr, size_t sz = 0, size_t nmax = 0){
			header = (ArrayHeader*)ptr;
			if(sz){
				header->sz = sz;
				header->nmax = nmax;
			}
			ptr += sizeof(ArrayHeader);
			buf	= ptr;
			ptr += header->sz * header->nmax;
		}

		// returns idx-th element
		T* Get(int idx){
			if(idx == -1)
				return 0;
			if(0 <= idx && idx < (int)header->nmax)
				return (T*)(buf + idx * header->sz);
			throw SceneInvalidOperation();
		}
		int	Create(){
			// find first element that is not in use
			for(uint i = 0; i < header->nmax; i++){
				byte* elem = (buf + i * header->sz);
				if(*elem == 0){
					*elem = 1;
					header->num++;
					return i;
				}
			}
			throw SceneFailure();
		}
		void Delete(int idx){
			if(0 <= idx && idx < (int)header->nmax){
				byte* elem = (buf + idx * header->sz);
				if(*elem == 1){
					*elem = 0;
					header->num--;
				}
			}
			else throw SceneInvalidOperation();
		}
		void DeleteAll(){
			byte* tail = buf + header->nmax * header->sz;
			for(byte* elem = buf; elem != tail; elem += header->sz)
				*elem = 0;
			header->num = 0;
		}
	};

	template<class T>
	struct List : Array< ListNode<T> >{
		/// adds a list node to list [first, last]
		void AddToList(int idx, int& first, int& last){
			ListNode<T>* node = this->Get(idx);
			if(first == -1 && last == -1){
				node->prev = -1;
				node->next = -1;
				first = idx;
				last  = idx;
			}
			else{
				node->prev = last;
				node->next = -1;
				ListNode<T>* lastNode = this->Get(last);
				lastNode->next = idx; 
				last = idx;
			}
		}
		/// removes a list node from list [first, last]
		void RemoveFromList(int idx, int& first, int& last){
			ListNode<T>* node = this->Get(idx);
			ListNode<T>* prevNode = this->Get(node->prev);
			ListNode<T>* nextNode = this->Get(node->next);

			// 消すノードの前をつなぎかえ
			if(prevNode){
				prevNode->next = node->next;
			}
			else{
				first = node->next;
				if(nextNode)
					nextNode->prev = -1;
			}
	
			// 消すノードの後をつなぎかえ
			if(nextNode){
				nextNode->prev = node->prev;
			}
			else{
				last = node->prev;
				if(prevNode)
					prevNode->next = -1;
			}
		}
		/// finds a list node in the list [first, ] that evaluates true with given predicate
		template<class P>
		int	FindInList(P pred, int first){
			int idx = first;
			while(idx != -1){
				ListNode<T>* node = this->Get(idx);
				if(pred(node->val))
					return idx;
				idx = node->next;
			}
			return -1;
		}
		/// deletes all list nodes in the list [first, ]
		void DeleteList(int first){
			int idx = first;
			while(idx != -1){
				ListNode<T>* node = this->Get(idx);
				this->Delete(idx);
				idx = node->next;
			}
		}
		/// store values of list nodes in array
		void GetValues(int first, vector<T>& vals){
			vals.clear();
			int idx = first;
			while(idx != -1){
				ListNode<T>* node = this->Get(idx);
				vals.push_back(node->val);
				idx = node->next;
			}
		}
	};

	typedef List<int>	ChildList;
	typedef List<Link>	LinkList;
	
public:
	string				sharedName;			///< shared memory name
	SharedMemory		sharedMemory;		///< shared memory object
	
	Header*						header;			///< pointer to Header struct in shared memory
	Array<Object>				objArray;		///< array of object nodes
	ChildList					childList;		///< list of parent-child relations
	LinkList					linkList;		///< list of links
	Array<Location>				locArray;		///< array of locations
	vector< Array<Property> >	propArray;		///< array of properties

public:	
	void InitArrays();
	void GetLivenessRecursively(int id, vector<byte>& alive);

public:
	/** @brief	creates shared memory
		@param	name	shared memory name
		@param	db		type db
		@param	conf	configuration
	 **/
	void	Create(const string& name, TypeDB* db, const Config& conf);
	
	/**	@brief	opens shared memory
		@param	name	shared memory name
		@param	db		type db
	 **/
	void	Open(const string& name, TypeDB* db);

	/**
	 **/
	bool	IsOpen();

	/** @brief	closes shared memory
	 **/
	void	Close();

	/** @brief	return shared memory name
	 **/
	string	GetSharedMemoryName(){ return sharedName; }

	/** @brief	print shared memory information
	 **/
	void		PrintInfo(ostream& os, TypeDB* typedb);

	/// TreeBase virtual functions
	virtual void		Clear      ();
	virtual int			GetRoot    ();
	virtual int			GetParent  (int id);
	virtual void		GetChildren(int id, vector<int>& children);
	virtual string		GetName    (int id);
	virtual void        SetName    (int id, string name);
	virtual void		AddChild   (int parId, int childId);
	virtual void		RemoveChild(int parId, int childId);

	/// SceneBase virtual functions
	virtual int			CreateObject    (int type, const string& name, TypeDB* typedb);
	virtual void		DeleteObject    (int id);
	virtual int			GetObjectType   (int id);
	virtual void		AddLink         (int srcId, int destId, const string& name);
	virtual void		RemoveLink      (int srcId, int destId);
	virtual int			GetLink         (int id, const string& name);
	virtual void		GetLinks        (int id, vector<int>& links, vector<string>& names);
	virtual Property*	GetProperty     (int id);
	virtual void		GetLivenessArray(Address& addr, vector<byte>& alive);
	virtual void		SetLocation     (int id, const string& path);
	virtual string		GetLocation     (int id);
};

}
