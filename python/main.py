#!/usr/bin/env python3
"""
扫描当前文件夹，将 ELF 文件转换为 BIN 和 C 数组
支持自动检测 objcopy 工具，提供多种提取策略
用法: python3 elf2c.py [选项]

选项:
  --output-dir DIR    指定输出目录（默认: ./c_arrays）
  --no-objcopy        强制使用手动解析（不使用 objcopy）
  --keep-sections     指定要保留的段（逗号分隔，默认: .text,.rodata,.data,.init,.fini）
  --full              提取所有 LOAD 段（等同于 objcopy -O binary）
  --help              显示帮助信息
"""

import os
import sys
import subprocess
import struct
import argparse
from pathlib import Path

def find_objcopy():
    """自动检测可用的 objcopy 工具"""
    candidates = [
        os.environ.get("OBJCOPY", ""),
        "arm-none-eabi-objcopy",
        "arm-linux-gnueabi-objcopy",
        "arm-eabi-objcopy",
        "riscv64-unknown-elf-objcopy",
        "riscv64-elf-objcopy",
        "mips-linux-gnu-objcopy",
        "objcopy",
    ]
    
    for c in candidates:
        if not c:
            continue
        try:
            result = subprocess.run(
                ["which", c], 
                capture_output=True, 
                text=True,
                check=False
            )
            if result.returncode == 0:
                return c
        except Exception:
            continue
    
    # 尝试直接运行
    for c in candidates:
        if not c:
            continue
        try:
            subprocess.run([c, "--version"], capture_output=True, check=True)
            return c
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue
    
    return None

def get_elf_arch(elf_path):
    """获取 ELF 文件架构信息"""
    try:
        with open(elf_path, 'rb') as f:
            data = f.read(64)
        
        if data[:4] != b'\x7fELF':
            return None
        
        ei_class = data[4]
        ei_data = data[5]
        ei_machine = struct.unpack('<H', data[18:20])[0] if ei_data == 1 else struct.unpack('>H', data[18:20])[0]
        
        arch_map = {
            40: "ARM",
            41: "ARM64",
            62: "x86_64",
            3: "x86",
            243: "RISC-V",
            8: "MIPS",
            20: "PowerPC",
        }
        
        return arch_map.get(ei_machine, f"Unknown({ei_machine})")
    except Exception:
        return None

def parse_elf_load_segments(elf_path):
    """手动解析 ELF 文件的 LOAD 段（降级方案）"""
    try:
        with open(elf_path, 'rb') as f:
            data = f.read()
        
        if data[:4] != b'\x7fELF':
            return None
        
        ei_class = data[4]
        ei_data = data[5]
        endian = '<' if ei_data == 1 else '>'
        
        segments = []
        
        if ei_class == 1:  # 32位
            e_phoff = struct.unpack(f'{endian}I', data[28:32])[0]
            e_phnum = struct.unpack(f'{endian}H', data[44:46])[0]
            e_phentsize = struct.unpack(f'{endian}H', data[42:44])[0]
            
            for i in range(e_phnum):
                ph_offset = e_phoff + i * e_phentsize
                if ph_offset + 32 > len(data):
                    break
                    
                p_type = struct.unpack(f'{endian}I', data[ph_offset:ph_offset+4])[0]
                
                if p_type == 1:  # PT_LOAD
                    p_offset = struct.unpack(f'{endian}I', data[ph_offset+4:ph_offset+8])[0]
                    p_vaddr = struct.unpack(f'{endian}I', data[ph_offset+8:ph_offset+12])[0]
                    p_filesz = struct.unpack(f'{endian}I', data[ph_offset+16:ph_offset+20])[0]
                    p_memsz = struct.unpack(f'{endian}I', data[ph_offset+20:ph_offset+24])[0]
                    p_flags = struct.unpack(f'{endian}I', data[ph_offset+24:ph_offset+28])[0]
                    
                    if p_filesz > 0:
                        seg_data = data[p_offset:p_offset+p_filesz]
                        segments.append({
                            'vaddr': p_vaddr,
                            'offset': p_offset,
                            'filesz': p_filesz,
                            'memsz': p_memsz,
                            'flags': p_flags,
                            'data': seg_data
                        })
        
        elif ei_class == 2:  # 64位
            e_phoff = struct.unpack(f'{endian}Q', data[32:40])[0]
            e_phnum = struct.unpack(f'{endian}H', data[56:58])[0]
            e_phentsize = struct.unpack(f'{endian}H', data[54:56])[0]
            
            for i in range(e_phnum):
                ph_offset = e_phoff + i * e_phentsize
                if ph_offset + 56 > len(data):
                    break
                    
                p_type = struct.unpack(f'{endian}I', data[ph_offset:ph_offset+4])[0]
                
                if p_type == 1:  # PT_LOAD
                    p_flags = struct.unpack(f'{endian}I', data[ph_offset+4:ph_offset+8])[0]
                    p_offset = struct.unpack(f'{endian}Q', data[ph_offset+8:ph_offset+16])[0]
                    p_vaddr = struct.unpack(f'{endian}Q', data[ph_offset+16:ph_offset+24])[0]
                    p_filesz = struct.unpack(f'{endian}Q', data[ph_offset+32:ph_offset+40])[0]
                    p_memsz = struct.unpack(f'{endian}Q', data[ph_offset+40:ph_offset+48])[0]
                    
                    if p_filesz > 0:
                        seg_data = data[p_offset:p_offset+p_filesz]
                        segments.append({
                            'vaddr': p_vaddr,
                            'offset': p_offset,
                            'filesz': p_filesz,
                            'memsz': p_memsz,
                            'flags': p_flags,
                            'data': seg_data
                        })
        
        if not segments:
            return None
        
        # 按虚拟地址排序
        segments.sort(key=lambda x: x['vaddr'])
        
        # 合并重叠或连续的段
        merged = []
        for seg in segments:
            if not merged:
                merged.append(seg)
            else:
                last = merged[-1]
                last_end = last['vaddr'] + last['memsz']
                if seg['vaddr'] <= last_end:
                    # 重叠或连续，合并
                    overlap = last_end - seg['vaddr']
                    if overlap > 0:
                        # 跳过重叠部分
                        seg_data = seg['data'][overlap:] if overlap < len(seg['data']) else b''
                    else:
                        seg_data = seg['data']
                    # 扩展最后一个段
                    if seg['vaddr'] + len(seg_data) > last['vaddr'] + len(last['data']):
                        padding = b'\x00' * (seg['vaddr'] - last_end)
                        last['data'] += padding + seg_data
                        last['memsz'] = seg['vaddr'] + seg['memsz'] - last['vaddr']
                else:
                    # 不连续，添加填充
                    padding = b'\x00' * (seg['vaddr'] - last_end)
                    last['data'] += padding
                    merged.append(seg)
        
        # 合并所有数据
        result = b''
        for seg in merged:
            result += seg['data']
        
        return result
        
    except Exception as e:
        print(f"  手动解析失败: {e}")
        return None

def extract_with_objcopy(elf_path, bin_path, objcopy, sections=None, full=False):
    """使用 objcopy 提取数据"""
    try:
        if full:
            # 提取所有 LOAD 段
            cmd = [objcopy, "-O", "binary", elf_path, bin_path]
        else:
            # 提取指定段
            cmd = [objcopy, "-O", "binary"]
            if sections:
                for section in sections.split(','):
                    section = section.strip()
                    if section:
                        cmd.extend(["-j", section])
            cmd.extend([elf_path, bin_path])
        
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        
        if result.returncode != 0:
            return False, result.stderr
        
        # 检查生成的文件
        if not os.path.exists(bin_path) or os.path.getsize(bin_path) == 0:
            return False, "生成的文件为空"
        
        return True, None
        
    except Exception as e:
        return False, str(e)

def elf_to_bin_and_h(elf_path, output_dir, objcopy=None, no_objcopy=False, 
                     sections=None, full=False, verbose=True):
    """将 ELF 文件转换为 BIN 和 C 头文件"""
    name = Path(elf_path).stem
    bin_path = Path(output_dir) / f"{name}.bin"
    h_path = Path(output_dir) / f"{name}.h"
    
    if verbose:
        print(f"处理: {Path(elf_path).name}")
    
    # 获取架构信息
    arch = get_elf_arch(elf_path)
    if arch and verbose:
        print(f"  架构: {arch}")
    
    bin_data = None
    method = None
    
    # 策略1: 使用 objcopy
    if not no_objcopy and objcopy:
        if verbose:
            print(f"  尝试使用: {objcopy}")
        
        success, error = extract_with_objcopy(elf_path, bin_path, objcopy, sections, full)
        
        if success:
            with open(bin_path, 'rb') as f:
                bin_data = f.read()
            method = f"objcopy ({objcopy})"
            if verbose:
                print(f"  ✅ 提取成功 ({len(bin_data)} 字节)")
        else:
            if verbose:
                print(f"  ⚠️ objcopy 失败: {error}")
    
    # 策略2: 手动解析
    if bin_data is None:
        if verbose:
            print("  尝试手动解析 ELF...")
        
        bin_data = parse_elf_load_segments(elf_path)
        
        if bin_data is not None:
            method = "手动解析"
            # 保存手动提取的 BIN
            with open(bin_path, 'wb') as f:
                f.write(bin_data)
            if verbose:
                print(f"  ✅ 手动解析成功 ({len(bin_data)} 字节)")
    
    # 策略3: 尝试使用系统的 objcopy（如果之前没找到）
    if bin_data is None and not no_objcopy:
        system_objcopy = find_objcopy()
        if system_objcopy and system_objcopy != objcopy:
            if verbose:
                print(f"  尝试使用系统 objcopy: {system_objcopy}")
            
            success, error = extract_with_objcopy(elf_path, bin_path, system_objcopy, sections, full)
            
            if success:
                with open(bin_path, 'rb') as f:
                    bin_data = f.read()
                method = f"objcopy ({system_objcopy})"
                if verbose:
                    print(f"  ✅ 提取成功 ({len(bin_data)} 字节)")
    
    # 如果所有方法都失败
    if bin_data is None or len(bin_data) == 0:
        print(f"  ❌ 无法提取数据，跳过")
        return False
    
    # 生成 C 头文件
    try:
        with open(h_path, 'w') as f:
            # 文件头
            f.write(f"#ifndef __{name.upper()}_H__\n")
            f.write(f"#define __{name.upper()}_H__\n\n")
            
            f.write(f"#include <stdint.h>\n")
            f.write(f"#include <stddef.h>\n\n")
            
            f.write(f"// ============================================\n")
            f.write(f"// 从: {Path(elf_path).name}\n")
            f.write(f"// 架构: {arch or 'Unknown'}\n")
            f.write(f"// 方法: {method}\n")
            f.write(f"// 大小: {len(bin_data)} 字节\n")
            f.write(f"// 生成时间: {__import__('datetime').datetime.now()}\n")
            f.write(f"// ============================================\n\n")
            
            f.write(f"#define {name.upper()}_SIZE {len(bin_data)}U\n\n")
            
            # 如果数据太大，添加注释
            if len(bin_data) > 1024 * 1024:  # 1MB
                f.write(f"// 警告: 数据较大 ({len(bin_data)} 字节)，建议使用 BIN 文件\n\n")
            
            # 数组定义
            f.write(f"static const uint8_t {name}_data[] = {{\n")
            
            for i in range(0, len(bin_data), 16):
                chunk = bin_data[i:i+16]
                hex_str = ', '.join(f'0x{b:02X}' for b in chunk)
                f.write(f"    {hex_str},\n")
            
            f.write(f"}};\n\n")
            
            f.write(f"static const size_t {name}_len = sizeof({name}_data);\n\n")
            
            # 可选的辅助宏
            f.write(f"#define {name.upper()}_DATA {name}_data\n")
            f.write(f"#define {name.upper()}_LEN  {name}_len\n\n")
            
            f.write(f"#endif // __{name.upper()}_H__\n")
        
        if verbose:
            print(f"  📝 生成头文件: {h_path}")
        return True
        
    except Exception as e:
        print(f"  ❌ 生成头文件失败: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="将 ELF 文件转换为 BIN 和 C 数组",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 elf2c.py                          # 处理当前目录所有 .elf 文件
  python3 elf2c.py --output-dir ./output    # 指定输出目录
  python3 elf2c.py --full                   # 提取所有 LOAD 段
  python3 elf2c.py --keep-sections .text,.data  # 只提取指定段
  python3 elf2c.py --no-objcopy             # 强制使用手动解析
        """
    )
    
    parser.add_argument('--output-dir', default='./c_arrays',
                       help='指定输出目录 (默认: ./c_arrays)')
    parser.add_argument('--no-objcopy', action='store_true',
                       help='强制使用手动解析（不使用 objcopy）')
    parser.add_argument('--keep-sections', default='.text,.rodata,.data,.init,.fini',
                       help='指定要保留的段，逗号分隔 (默认: .text,.rodata,.data,.init,.fini)')
    parser.add_argument('--full', action='store_true',
                       help='提取所有 LOAD 段（等同于 objcopy -O binary）')
    parser.add_argument('--verbose', action='store_true', default=True,
                       help='显示详细信息 (默认)')
    parser.add_argument('--quiet', action='store_true',
                       help='静默模式，不显示详细信息')
    parser.add_argument('--elf', help='指定单个 ELF 文件处理')
    
    args = parser.parse_args()
    
    # 设置详细模式
    verbose = not args.quiet
    
    # 创建输出目录
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 检测 objcopy
    objcopy = None if args.no_objcopy else find_objcopy()
    
    if verbose:
        print("=" * 60)
        print("ELF 转 BIN/C 数组工具")
        print("=" * 60)
        if objcopy:
            print(f"检测到 objcopy: {objcopy}")
        else:
            print("未检测到 objcopy，将使用手动解析")
        print(f"输出目录: {output_dir}")
        print(f"保留段: {args.keep_sections if not args.full else '所有 LOAD 段'}")
        print("-" * 60)
    
    # 查找 ELF 文件
    if args.elf:
        elf_files = [Path(args.elf)]
        if not elf_files[0].exists():
            print(f"❌ 文件不存在: {args.elf}")
            sys.exit(1)
    else:
        current_dir = Path.cwd()
        elf_files = list(current_dir.glob("*.elf"))
        if not elf_files:
            print("❌ 未找到 .elf 文件")
            sys.exit(1)
    
    if verbose:
        print(f"找到 {len(elf_files)} 个 ELF 文件\n")
    
    # 处理每个 ELF 文件
    success_count = 0
    for elf_path in elf_files:
        success = elf_to_bin_and_h(
            elf_path=elf_path,
            output_dir=output_dir,
            objcopy=objcopy,
            no_objcopy=args.no_objcopy,
            sections=None if args.full else args.keep_sections,
            full=args.full,
            verbose=verbose
        )
        if success:
            success_count += 1
        if verbose:
            print()
    
    # 统计结果
    print("=" * 60)
    print(f"✨ 完成！")
    print(f"   处理: {len(elf_files)} 个文件")
    print(f"   成功: {success_count} 个")
    print(f"   输出: {output_dir}")
    print("=" * 60)
    
    # 列出生成的文件
    if success_count > 0:
        print("\n生成的文件:")
        for ext in ['*.bin', '*.h']:
            files = list(output_dir.glob(ext))
            if files:
                print(f"  {ext}:")
                for f in sorted(files):
                    size = f.stat().st_size
                    print(f"    {f.name} ({size:,} 字节)")

if __name__ == "__main__":
    main()