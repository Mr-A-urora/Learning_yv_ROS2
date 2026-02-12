#include <iostream>
#include <thread>   // 多线程相关
#include <chrono>   // 时间相关
#include <functional>   // 函数包装器
#include "cpp-httplib/httplib.h"   //下载相关

using namespace std;

class Download{
private:

public:
    void download(const string& host, const string& path, const function<void(const string&, const string&)>& callback_word_count){
        cout << "线程" << this_thread::get_id() << endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200){
            callback_word_count(path, response->body);
        } else {
            cout << "下载失败: " << path << endl;
        }
    };

    void start_download(const string& host, const string& path, const function<void(const string&, const string&)>& callback_word_count){
        auto download_fun = bind(&Download::download, this, placeholders::_1, placeholders::_2, placeholders::_3);
        thread thread(download_fun, host, path, callback_word_count);
        thread.detach();   // 分离线程
    };
};

int main(){
    auto d = Download();
    auto word_count_callback = [](const string& path, const string& result) -> void {
        cout << "下载完成" << path << "->" << result.length() << "->" << result.substr(0,9)<< endl;
    };

    d.start_download("http://0.0.0.0:8000", "/novel1.txt", word_count_callback);
    d.start_download("http://0.0.0.0:8000", "/novel2.txt", word_count_callback);
    d.start_download("http://0.0.0.0:8000", "/novel3.txt", word_count_callback);

    this_thread::sleep_for(chrono::milliseconds(1000 * 10));   // 休眠10s

    return 0;
}
