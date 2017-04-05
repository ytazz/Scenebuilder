#include <sblogger.h>
#include <sbmessage.h>

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

LogBase::LogBase(){
	binary = false;
}

void LogBase::Clear(){
	infos.clear();
}

void LogBase::Set(SceneBase* s, TypeDB* db, int id){
	scene  = s;
	typedb = db;
	rootId = id;
}

void LogBase::SetBinary(bool on){
	binary = on;
}

void LogBase::AddRecurs(int id){
	Add(id);
	vector<int> children;
	scene->GetChildren(id, children);
	for(uint i = 0; i < children.size(); i++)
		AddRecurs(children[i]);
}

void LogBase::Add(int id){
	int typeId = scene->GetObjectType(id);	
	if(typeId == BodyProp::id || typedb->KindOf(typeId, Joint1DProp::id)){
		ObjectInfo obj;
		obj.id     = id;
		obj.type   = typedb->GetType(typeId);
		obj.nbytes = obj.type->CalcBytes(AttrCategory::State);
		scene->GetAddress(obj.addr, rootId, id);
		infos.push_back(obj);
	}
}

//-------------------------------------------------------------------------------------------------

void LogReader::ParseHeader(){
	string_iterator_pair str;
	
	nbytesHeader = 0;
	for(tok.Set(contents, "\n", false); !tok.IsEnd(); tok.Next()){
		str = tok.GetToken();

		// ＊テキストモードのストリームが読み込んだ時点で改行文字は処理系によらず1byteになっている
		nbytesHeader += (uint)str.size() + 1;

		// 空行が来たらヘッダ終了
		if(str == ""){
			ptr     = (const byte*)&contents[0] + nbytesHeader;
			ptr_end = (const byte*)&contents[0] + contents.size();
			break;
		}

		// 文字列をアドレスに変換して格納
		infos.push_back(LogBase::ObjectInfo());
		infos.back().addr.FromString(string(str.begin(), str.end()));
	}

	// アドレスからオブジェクトIDとタイプIDを取得
	nbytes = 0;
	for(uint i = 0; i < infos.size(); i++){
		LogBase::ObjectInfo& obj = infos[i];

		try{
			obj.id     = scene->Find(obj.addr, rootId);
			obj.type   = typedb->GetType(scene->GetObjectType(obj.id));
			obj.nbytes = obj.type->CalcBytes(AttrCategory::State);
			nbytes += obj.nbytes;
		}
		catch(Exception&){
			Message::Error("invalid path %s in log header", obj.addr.ToString().c_str());
		}
	}
}

void LogReader::ParseBinary(){
	while(ptr + sizeof(real_t) + nbytes <= ptr_end){
		LogReader::Snapshot* snap = new LogReader::Snapshot();
		snap->time = *(real_t*)ptr;
		ptr += sizeof(real_t);

		for(uint i = 0; i < infos.size(); i++){
			snap->props.push_back(ptr);
			ptr += infos[i].nbytes;
		}
		
		snapshots.push_back(snap);
	}
}

void LogReader::ParseText(){
	bool stamp = true;
	string_iterator_pair str;
	
	for( ; !tok.IsEnd(); tok.Next()){
		str = tok.GetToken();
		
		if(stamp){
			if(str == "")
				continue;

			// タイムスタンプを格納してデータセクションへ
			snapshots.push_back(new LogReader::Snapshot());
			snapshots.back()->time = to_real(str);
			
			stamp = false;
		}
		else{
			if(str == ""){
				// 空行が来たら次のスナップショットのタイムスタンプへ
				stamp = true;
			}
			else{
				// データはRead呼び出し時にパースする．この段階では文字列の範囲のみ記憶しておく
				snapshots.back()->ranges.push_back(str);
			}
		}
	}
}

void LogReader::Load(const char* filename){
	ifstream file(filename, ios_base::binary);
	if(!file.is_open())
		throw FileError();

	infos.clear();
	snapshots.clear();

	// 中身を全部吸出し
	file.seekg(0, file.end);
	int len = (int)file.tellg();
	file.seekg(0, file.beg);

	contents.resize(len);
	file.read(&contents[0], len);
	
	file.close();

	// 解析
	ParseHeader();
	if(binary)
		 ParseBinary();
	else ParseText();	
}

int LogReader::Count(){
	return (int)snapshots.size();
}

int LogReader::Nearest(real_t time){
	if(time < snapshots[0]->time)
		return 0;
	for(int i = 0; i < Count()-1; i++){
		if(snapshots[i]->time <= time && time <= snapshots[i+1]->time)
			return i;
	}
	return Count()-1;
}

real_t LogReader::Time(int idx){
	if(idx < 0 || idx >= Count()){
		Message::Error("LogReader: index out of range");
		return 0;
	}
	return snapshots[idx]->time;
}

void LogReader::Read(int idx){
	if(idx < 0 || idx >= Count()){
		Message::Error("LogReader: index out of range");
		return;
	}
	Snapshot* snap = snapshots[idx];

	// 各オブジェクトについて型情報をもとに文字列をパースしてプロパティを設定
	int nObj = (int)infos.size();
	for(int i = 0; i < nObj; i++){
		// シーンに対応するオブジェクトがない場合はスキップ
		if(infos[i].id == -1)
			continue;
		Property* prop = scene->GetProperty(infos[i].id);

		if(binary){
			// FromBinaryがポインタを進めるので一時変数に入れてから渡す
			const byte* tmp = snap->props[i];
			infos[i].type->FromBinary(tmp, prop, AttrCategory::State);
		}
		else{
			infos[i].type->FromString(snap->ranges[i], prop, AttrCategory::State);
		}
	}
}

//-------------------------------------------------------------------------------------------------

LogWriter::LogWriter(){

}

void LogWriter::Open(const char* filename){
	Close();
	logFile.open(filename, ofstream::binary);
	
	WriteHeader();
}

void LogWriter::Close(){
	logFile.close();
}

void LogWriter::WriteHeader(){
	// 各オブジェクトのアドレス
	for(uint i = 0; i < infos.size(); i++){
		logFile << infos[i].addr.ToString() << endl;
	}
	// 空行
	logFile << endl;

	// 総バイト数計算
	size_t nbytes = 0;
	for(uint i = 0; i < infos.size(); i++)
		nbytes += infos[i].nbytes;

	buf.resize(nbytes, 0);
}

void LogWriter::Write(real_t time){
	if(binary)
		 WriteBinary(time);
	else WriteText(time);
}

void LogWriter::WriteBinary(real_t time){
	byte* ptr = &buf[0];

	for(uint i = 0; i < infos.size(); i++){
		ObjectInfo& obj = infos[i];
		obj.type->ToBinary(ptr, scene->GetProperty(obj.id), AttrCategory::State);
	}

	logFile.write((const char*)&time, sizeof(real_t));
	logFile.write((const char*)&buf[0], buf.size());
	logFile.flush();
}

void LogWriter::WriteText(real_t time){
	// タイムスタンプ
	logFile << time << endl;
	// 各オブジェクトのプロパティ（保存レベル以上）
	for(uint i = 0; i < infos.size(); i++){
		ObjectInfo& obj = infos[i];

		obj.type->ToString(logFile, scene->GetProperty(obj.id), AttrCategory::State);
		logFile << endl;
	}
	// 空行
	logFile << endl;
	logFile.flush();
}

}
