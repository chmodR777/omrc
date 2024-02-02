#pragma once

/**
* 主要用于处理：当 operator new 分配内存失败时（主要是stl、第三方库中会出现），比如分配的内存过大，或者系统内存不足等情况
* 能够在抛出 std::bad_alloc 异常 前 输出当前调用栈 & 当前进程内存占用情况，方便定位问题。
* 一旦设置，整个进程内都有效
*/
void setNewHandler();
