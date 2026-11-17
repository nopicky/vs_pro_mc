# 维控 C++ 编码规范（v0.1）

> 来源：`old4wt/encoder_test/encoder/维控C++编码规范.pdf`（6 页，四章）。
> 本文是原文的**可执行版**：把叙述压成规则，补上项目实例对照。原文没覆盖的实践单独标出。
> **写 C/C++ 代码前必读**，尤其是新建文件、加函数、改结构体的时候。

## 1. 命名风格三种（概念区分）

| 风格 | 说明 | 例 |
|---|---|---|
| 大驼峰 UpperCamelCase | 首字母大写 | `CFileTool` |
| 小驼峰 lowerCamelCase | 首字母小写 | `createFile` |
| 内核 unix_like | 全小写下划线 | `test_result` |
| 匈牙利 | 大驼峰 + 类型/用途前缀 | `uiSavedCount` |

⚠️ 注意：规范的示例代码里类成员用了小驼峰（`string sFileName; int nFileSize;`），前缀是小写 + 大驼峰词干。**跟随示例的实际写法**。

## 2. 各类命名（硬规则）

| 对象 | 规则 | 例 |
|---|---|---|
| **变量** | 匈牙利风格 | `int nFileSize; string sFilePath; char *pFileName;` |
| **函数** | 小驼峰，优先「动词+名词」 | `createFile(); getUrl(); printResult();` |
| **源文件** | 大驼峰，与内部类名一致，**去掉 C 前缀** | 类 `CFileTool` → `FileTool.cpp` / `FileTool.h` |
| **类名** | 大驼峰，**统一加 C 前缀** | `class CFileTool {}` |
| **宏** | 全大写 + 下划线 | `#define MAX_PATH (260)` |
| **结构体** | 全大写 + 下划线，**后缀 `_T`** | `typedef struct tagFILE_INFO_T {...} FILE_INFO_T;` |

### 匈牙利前缀表（背下来）

**属性部分**

| 前缀 | 含义 | 前缀 | 含义 |
|---|---|---|---|
| `g_` | 全局变量 | `c_` | 常量 |
| `m_` | 成员变量 | `s_` | 静态变量 |

**类型部分**

| 前缀 | 含义 | 前缀 | 含义 |
|---|---|---|---|
| `a` | 数组 | `p` | 指针 |
| `fn` | 函数 | `v` | 无效（void） |
| `h` | 句柄 | `n` | 整型 |
| `b` | 布尔 | `f` | 浮点型 |
| `sz` | C 字符串 | `by` | 字节 |
| `s` | string | `vec` | vector |
| `list` | list | `map` | map |

### 头文件防御宏

格式：**文件名按单词分割 → 全大写下划线拼接 → `H` → 两个下划线 → 当前日期**

```c
#ifndef FILE_TOOL_H__20210513
#define FILE_TOOL_H__20210513

#endif /* FILE_TOOL_H__20210513 */
```

## 3. 排版（K&R 风格）

左大括号跟随语句放**行末**，右大括号**独占一行**。空函数体可同行。

| 规则 | 说明 |
|---|---|
| **if 必须带大括号** | 哪怕只有一条语句 |
| **禁止 if/else 写同一行** | `} else {` 里的 else 不能和 if 同行 |
| **for/while 必须带大括号** | 空循环体写 `{}`，**不许用单个分号 `;`** |
| **switch 的 case/default 缩进一层** | 见下方样板 |
| **函数定义 K&R，首行空一行** | 见下方样板 |

```c
/* switch 样板 */
switch (var) {
    case 0: {
        doSomething1();
    } break;

    case 1: {
        doSomething2();
    } break;

    default: {
        ...
    } break;
}

/* 函数定义样板：首行空一行 */
int getFileSize(const string& sFilePath) {

    int nFileSize;
    ...
}
```

## 4. 注释

**统一用 `/* */`**（不要 `//`），内容**建议英文**——规避中文乱码。

| 规则 | 说明 |
|---|---|
| 函数头注释位置 | 声明或定义**上方** |
| 函数头注释取舍 | 尽量靠函数名自注释；**不写冗余的、空有格式的**函数头 |
| 代码注释位置 | 优先**上方**，特殊情况才右侧 |
| 注释符与内容 | 间隔 **1 空格** |
| 右置注释 | 与前面代码至少 1 空格；同类可上下对齐 |
| 上方注释 | 与对应代码**同缩进** |
| 条件编译 | 结束行右方加 `#endif /* FLAG_XXX */` |
| 全局变量 | **详细注释**：功能、取值范围、哪些函数/过程存取、注意事项 |
| 数据结构 | 声明注释放**上方相邻**位置（不是下面）；结构体**每个域的注释放右方** |

```c
/* get filesize */
int getFileSize(const string& sFilePath);

/* write string
   return writed length */
int writeString(const char *pBuf, int nLen);

/* 右置注释示例 */
const int c_Const = 100;        /* 相关的同类注释，可以考虑上下对齐 */
const int c_AnotherConst = 200; /* 上下对齐时，与左侧代码保持间隔 */
```

## 5. 与项目现状的对照（重要）

### 一致的部分
- 项目代码大量使用 `_T` 后缀结构体（`CPLD_4WT_MODULE_T`、`WT4_MODULE_T`）→ 符合规范
- 全局变量带 `g_` 前缀（`g_auWt4DbgForceBadFrame`）→ 符合规范
- 宏全大写（`WT4_FRAME_RAW_BASE`、`MAX_PATH`）→ 符合规范

### 需要留意的差异
- **格式化工具**：项目 `.vscode/settings.json` 配的是 `{BasedOnStyle: LLVM, ColumnLimit: 120}`。
  LLVM 本身是 K&R 变体（左括号不换行），基本兼容；但 clang-format 不管注释用 `//` 还是 `/* */`，
  **注释形式靠自觉**，别指望格式化工具兜底。
- **规范未覆盖的前缀**：实际代码里常见 `u`（unsigned）、`u8`/`u16`/`u32`（定宽无符号）、`e`（enum）等，
  v0.1 的前缀表里没有。跟随所在文件既有写法，**同一文件内保持一致**优先。
- **C / C++ 混编**：项目里 `.c` 和 `.cpp` 并存，接口用 `2C` / `Cpp2C` 后缀表示 C++ 实现向 C 导出。
  规范只针对 C++，写 `.c` 文件时命名规则照用，但别用 string/vector 那套。

## 6. 自检清单（提交前过一遍）

- [ ] 变量有匈牙利前缀？全局 `g_`、成员 `m_`、常量 `c_`
- [ ] 函数是小驼峰、动词开头？
- [ ] 新建文件的构造函数：类 `CXXX` → 文件 `XXX.cpp` / `XXX.h`
- [ ] 头文件有 `FILENAME_H__YYMMDD` 防御宏？
- [ ] 结构体名 `XXX_T` 结尾、`typedef struct tagXXX_T`？
- [ ] 所有 if / for / while 都有大括号？
- [ ] `else` 没跟 `if` 挤在同一行？
- [ ] 左大括号在行末、右大括号独占一行？
- [ ] 注释用 `/* */`、注释符后 1 空格？
- [ ] 全局变量写了详细注释（范围 + 存取者 + 注意事项）？
- [ ] 结构体每个域右侧有注释？
- [ ] `#endif` 右边标了对应的宏名？
