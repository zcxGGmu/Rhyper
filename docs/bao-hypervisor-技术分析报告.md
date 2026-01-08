# Bao Hypervisor 技术分析报告

## 目录

1. [项目概述](#项目概述)
2. [架构设计](#架构设计)
3. [核心组件分析](#核心组件分析)
4. [支持的架构](#支持的架构)
5. [内存管理](#内存管理)
6. [中断处理](#中断处理)
7. [虚拟化支持](#虚拟化支持)
8. [平台支持](#平台支持)
9. [构建系统](#构建系统)
10. [关键特性与设计哲学](#关键特性与设计哲学)

---

## 项目概述

### 基本信息

**Bao**（源自中文"保护"）是一个轻量级、开源的嵌入式 hypervisor，旨在提供强隔离性和实时性保证。Bao 提供了一个最小化的、从零开始实现的分区化 hypervisor 架构。

### 设计目标

1. **强隔离性**：为故障遏制提供隔离保障
2. **实时性**：确保实时行为和确定性
3. **最小化 TCB**（可信计算基）：不依赖不可信的大型单体通用操作系统
4. **静态分区**：资源在 VM 实例化时静态分区和分配

### 项目特点

- 无外部依赖（如 Linux 等特权 VM）
- 极小的代码量和内存占用
- 支持 ARMv8-A、ARMv8-R 和 RISC-V 架构
- 开源协议：Apache-2.0

---

## 架构设计

### 分区化 Hypervisor 架构

Bao 采用静态分区化 hypervisor 架构，其核心设计理念包括：

#### 1. 静态资源分区

```
┌─────────────────────────────────────────────────────────┐
│                    Bao Hypervisor                        │
│                  (Type-1, Bare Metal)                    │
├─────────────────────────────────────────────────────────┤
│  VM 0 (Linux)    │  VM 1 (RTOS)   │  VM 2 (Bare Metal) │
│  [静态分配]       │  [静态分配]     │  [静态分配]         │
└─────────────────────────────────────────────────────────┘
```

- **内存**：使用两级转换静态分配
- **I/O**：仅支持透传模式
- **中断**：虚拟中断直接映射到物理中断
- **CPU**：1:1 虚拟 CPU 到物理 CPU 映射，无需调度器

#### 2. 分层架构

```
┌─────────────────────────────────────┐
│      Guest OS / Bare Metal Apps    │
├─────────────────────────────────────┤
│         Virtualization Layer       │
│  (Stage-2 Translation, vGIC, etc.) │
├─────────────────────────────────────┤
│        Bao Hypervisor Core         │
│  (VM Management, IPC, Shmem)      │
├─────────────────────────────────────┤
│    Architecture Abstraction Layer  │
│  (ARMv8-A, ARMv8-R, RISC-V)       │
├─────────────────────────────────────┤
│         Hardware Platform          │
└─────────────────────────────────────┘
```

### 核心设计原则

1. **最小化特权软件**：仅实现虚拟化必需的功能
2. **静态配置**：编译时确定所有资源分配
3. **无调度开销**：1:1 CPU 映射消除上下文切换开销
4. **透传优先**：I/O 和中断直接透传，减少虚拟化开销

---

## 核心组件分析

### 1. 初始化流程 ([`init.c`](hyper/bao-hypervisor/src/core/init.c:16))

```c
void init(cpuid_t cpu_id)
{
    // 1. CPU 初始化（必须最先执行）
    cpu_init(cpu_id);
    
    // 2. 内存初始化（必须最先执行）
    mem_init();
    
    // 3. 平台特定初始化
    platform_init();
    
    // 4. 控制台初始化
    console_init();
    
    // 5. 中断初始化
    interrupts_init();
    
    // 6. VMM 初始化
    vmm_init();
}
```

**初始化顺序的重要性：**
- CPU 和内存初始化必须在最前面，因为其他组件依赖它们
- 使用同步屏障确保多核一致性
- 主核负责全局初始化，从核等待同步

### 2. 虚拟机管理 ([`vm.c`](hyper/bao-hypervisor/src/core/vm.c:14))

#### VM 结构体

```c
struct vm {
    vmid_t id;                           // VM ID
    const struct vm_config* config;       // 配置信息
    spinlock_t lock;                     // 自旋锁
    struct cpu_synctoken sync;           // CPU 同步令牌
    cpuid_t master;                      // 主 CPU ID
    
    struct vcpu* vcpus;                  // VCPU 数组
    size_t cpu_num;                      // CPU 数量
    cpumap_t cpus;                       // CPU 位图
    
    struct addr_space as;                // 地址空间
    struct vm_arch arch;                 // 架构特定数据
    
    struct list emul_mem_list;           // 内存模拟列表
    struct list emul_reg_list;           // 寄存器模拟列表
    
    struct vm_io io;                     // I/O 管理
    BITMAP_ALLOC(interrupt_bitmap, MAX_GUEST_INTERRUPTS);
    
    size_t ipc_num;                      // IPC 通道数
    struct ipc* ipcs;                    // IPC 通道数组
    
    size_t remio_dev_num;                // 远程 I/O 设备数
    struct remio_dev* remio_devs;        // 远程 I/O 设备数组
};
```

#### VCPU 结构体

```c
struct vcpu {
    node_t node;
    struct arch_regs regs;               // 架构寄存器
    struct vcpu_arch arch;              // 架构特定数据
    
    vcpuid_t id;                        // VCPU ID
    cpuid_t phys_id;                    // 物理 CPU ID
    bool active;                        // 活动状态
    
    struct vm* vm;                      // 所属 VM
};
```

#### VM 初始化流程

```c
struct vm* vm_init(struct vm_allocation* vm_alloc, 
                   struct cpu_synctoken* vm_init_sync,
                   const struct vm_config* vm_config, 
                   bool master, vmid_t vm_id)
{
    // 1. 初始化 VM 结构（仅主核）
    if (master) {
        vm_master_init(vm, vm_config, vm_id);
    }
    
    cpu_sync_barrier(vm_init_sync);
    
    // 2. 初始化每个核心
    vm_cpu_init(vm);
    
    cpu_sync_barrier(&vm->sync);
    
    // 3. 初始化每个虚拟核心
    vm_vcpu_init(vm, vm_config);
    
    cpu_sync_barrier(&vm->sync);
    
    // 4. 初始化内存保护（仅主核）
    if (master) {
        vm_mem_prot_init(vm, vm_config);
    }
    
    cpu_sync_barrier(&vm->sync);
    
    // 5. 架构相关初始化
    vm_arch_init(vm, vm_config);
    
    // 6. 创建 VM 地址空间（仅主核）
    if (master) {
        vm_init_mem_regions(vm, vm_config);
        vm_init_dev(vm, vm_config);
        vm_init_ipc(vm, vm_config);
        vm_init_remio(vm, vm_config);
    }
    
    cpu_sync_and_clear_msgs(&vm->sync);
    
    return vm;
}
```

### 3. VMM 管理 ([`vmm.c`](hyper/bao-hypervisor/src/core/vmm.c:127))

#### VMM 初始化

```c
void vmm_init()
{
    vmm_arch_init();
    vmm_io_init();
    shmem_init();
    remio_init();
    
    // 主核初始化 VM 分配结构
    if (cpu_is_master()) {
        for (size_t i = 0; i < CONFIG_VM_NUM; i++) {
            vm_assign[i].lock = SPINLOCK_INITVAL;
            cpu_sync_init(&vm_assign[i].root_sync, 
                         config.vmlist[i].platform.cpu_num);
        }
    }
    
    cpu_sync_barrier(&cpu_glb_sync);
    
    // 分配 VCPU 到 VM
    bool master = false;
    vmid_t vm_id = INVALID_VMID;
    if (vmm_assign_vcpu(&master, &vm_id)) {
        struct vm_allocation* vm_alloc = vmm_alloc_install_vm(vm_id, master);
        struct vm_config* vm_config = &config.vmlist[vm_id];
        struct vm* vm = vm_init(vm_alloc, &vm_assign[vm_id].root_sync, 
                               vm_config, master, vm_id);
        cpu_sync_barrier(&vm->sync);
        vcpu_run(cpu()->vcpu);
    } else {
        cpu_powerdown();
    }
}
```

#### VCPU 分配策略

1. **亲和性优先**：首先根据 CPU 亲和性分配
2. **按需分配**：未分配的 CPU 按需分配给需要 CPU 的 VM
3. **主核选择**：每个 VM 的第一个 CPU 被指定为主核

---

## 支持的架构

### ARMv8-A AArch64

**支持平台：**
- Xilinx Zynq UltraScale+ MPSoC ZCU102/4
- Ultra96 Zynq UltraScale+ ZU3EG
- NXP MCIMX8QM-CPU
- NVIDIA Jetson TX2
- 96Boards HiKey 960
- Raspberry Pi 4
- QEMU virt
- Arm Fixed Virtual Platforms
- Toradex Verdin iMX8M Plus

**关键特性：**
- GICv2/GICv3 中断控制器支持
- 虚拟 GIC (vGIC) 实现
- Stage-2 页表转换
- PSCI 支持
- SMMUv2 IOMMU 支持

### ARMv8-R AArch64/AArch32

**支持平台：**
- Arm Fixed Virtual Platforms
- Arm MPS3-AN536

**关键特性：**
- MPU 内存保护
- Cortex-R52 支持
- 实时性保证

### RISC-V RV64/RV32

**支持平台：**
- QEMU virt

**关键特性：**
- H 扩展虚拟化支持
- SBI (Supervisor Binary Interface) 支持
- AIA (Advanced Interrupt Architecture) 支持
- Stage-2 页表转换

---

## 内存管理

### 内存架构 ([`mem.c`](hyper/bao-hypervisor/src/core/mem.c:487))

#### 页池管理

```c
struct page_pool {
    paddr_t base;              // 基地址
    size_t num_pages;          // 页数
    size_t free;               // 空闲页数
    size_t last;               // 最后分配位置
    bitmap_t* bitmap;          // 位图
    spinlock_t lock;           // 自旋锁
    node_t node;               // 链表节点
};
```

#### 内存初始化流程

```
1. mem_prot_init() - 内存保护初始化
   ↓
2. cache_enumerate() - 缓存枚举（主核）
   ↓
3. mem_setup_root_pool() - 设置根页池
   ↓
4. config_init() - 配置初始化
   ↓
5. mem_reserve_physical_memory() - 预留物理内存
   ↓
6. mem_create_ppools() - 创建额外页池
   ↓
7. mem_color_hypervisor() - Hypervisor 着色（可选）
```

#### 内存分配策略

1. **页池分配**：
   - 支持连续页分配
   - 支持对齐分配
   - 支持颜色分配（用于缓存分区）

2. **静态预留**：
   - Hypervisor 镜像
   - CPU 启动栈
   - VM 镜像
   - 共享内存
   - 设备内存

3. **两级转换**：
   - Stage-1: Guest 物理地址 → Guest 虚拟地址
   - Stage-2: Guest 物理地址 → Host 物理地址

### 内存保护机制

#### MMU 模式 ([`src/core/mmu/`](hyper/bao-hypervisor/src/core/mmu/))

- 页表管理
- 内存映射
- 访问权限控制

#### MPU 模式 ([`src/core/mpu/`](hyper/bao-hypervisor/src/core/mpu/))

- 区域管理
- 访问权限控制
- 适用于 ARMv8-R

---

## 中断处理

### 中断架构 ([`interrupts.c`](hyper/bao-hypervisor/src/core/interrupts.c:62))

#### 中断初始化

```c
void interrupts_init(void)
{
    interrupts_arch_init();           // 架构特定初始化
    interrupts_arch_ipi_init();       // IPI 初始化
    cpu_sync_barrier(&cpu_glb_sync);  // 同步屏障
    interrupts_cpu_enable(interrupts_ipi_id, true);  // 启用 IPI
}
```

#### 中断处理流程

```
中断触发
    ↓
interrupts_handle(int_id)
    ↓
检查中断归属
    ├─ VM 中断 → vcpu_inject_hw_irq() → 转发到 VM
    ├─ Hypervisor 中断 → interrupt_handlers[int_id]() → Hypervisor 处理
    └─ 未知中断 → ERROR
```

### ARM 中断处理 ([`arch/armv8/interrupts.c`](hyper/bao-hypervisor/src/arch/armv8/interrupts.c:20))

#### GIC 初始化

```c
void interrupts_arch_init()
{
    gic_init();  // 初始化 GIC 控制器
}
```

#### 中断路由

- **GICv2**: 使用目标寄存器设置目标 CPU
- **GICv3**: 使用路由器设置目标 CPU (基于 MPIDR)

#### 虚拟 GIC (vGIC)

- [`vgic.c`](hyper/bao-hypervisor/src/arch/armv8/vgic.c) - vGIC 核心实现
- [`vgicv2.c`](hyper/bao-hypervisor/src/arch/armv8/vgicv2.c) - GICv2 虚拟化
- [`vgicv3.c`](hyper/bao-hypervisor/src/arch/armv8/vgicv3.c) - GICv3 虚拟化

### RISC-V 中断处理

#### AIA 支持

- PLIC (Platform-Level Interrupt Controller)
- IMSIC (Incoming MSI Controller)
- APLIC (Advanced Platform-Level Interrupt Controller)

---

## 虚拟化支持

### ARMv8 虚拟化

#### Hypervisor 配置寄存器 ([`arch/armv8/vmm.c`](hyper/bao-hypervisor/src/arch/armv8/vmm.c:10))

```c
void vmm_arch_init()
{
    vmm_arch_profile_init();
    
    uint64_t hcr = HCR_VM_BIT | HCR_IMO_BIT | HCR_FMO_BIT | HCR_TSC_BIT;
    
    if (DEFINED(AARCH64)) {
        hcr |= HCR_RW_BIT | HCR_APK_BIT | HCR_API_BIT;
    }
    
    sysreg_hcr_el2_write(hcr);  // 配置 HCR_EL2
    sysreg_cptr_el2_write(0);    // 配置 CPTR_EL2
}
```

**HCR_EL2 位说明：**
- `VM`: Stage-2 转换使能
- `IMO/FMO`: IRQ/FIQ 路由到 EL2
- `TSC`: 访问计数器陷阱
- `RW`: EL1 执行状态 (AArch64/AAArch32)
- `APK/API`: 指针认证陷阱

#### VCPU 初始化 ([`arch/armv8/vm.c`](hyper/bao-hypervisor/src/arch/armv8/vm.c:50))

```c
void vcpu_arch_init(struct vcpu* vcpu, struct vm* vm)
{
    vcpu->arch.vmpidr = vm_cpuid_to_mpidr(vm, vcpu->id);
    sysreg_vmpidr_el2_write(vcpu->arch.vmpidr);
    
    vcpu->arch.psci_ctx.state = vcpu->id == 0 ? ON : OFF;
    vcpu->arch.psci_ctx.lock = SPINLOCK_INITVAL;
    
    vgic_cpu_init(vcpu);  // 初始化 vGIC
}
```

#### VCPU 重置 ([`arch/armv8/vm.c`](hyper/bao-hypervisor/src/arch/armv8/vm.c:61))

```c
void vcpu_arch_reset(struct vcpu* vcpu, vaddr_t entry)
{
    memset(&vcpu->regs, 0, sizeof(struct arch_regs));
    
    vcpu_subarch_reset(vcpu);
    vcpu_writepc(vcpu, entry);
    
    sysreg_cntvoff_el2_write(0);
    
    // 设置已知状态的寄存器
    sysreg_sctlr_el1_write(SCTLR_RES1);
    sysreg_cntkctl_el1_write(0);
    sysreg_pmcr_el0_write(0);
}
```

### RISC-V 虚拟化

#### H 扩展配置 ([`arch/riscv/vm.c`](hyper/bao-hypervisor/src/arch/riscv/vm.c:14))

```c
void vm_arch_init(struct vm* vm, const struct vm_config* vm_config)
{
    paddr_t root_pt_pa;
    mem_translate(&cpu()->as, (vaddr_t)vm->as.pt.root, &root_pt_pa);
    
    // 配置 hgatp (Guest Address Translation and Protection)
    unsigned long hgatp = (root_pt_pa >> PAGE_SHIFT) | 
                         (HGATP_MODE_DFLT) |
                         ((vm->id << HGATP_VMID_OFF) & HGATP_VMID_MSK);
    
    csrs_hgatp_write(hgatp);
    
    virqc_init(vm, &vm_config->platform.arch.irqc);
}
```

#### VCPU 初始化 ([`arch/riscv/vm.c`](hyper/bao-hypervisor/src/arch/riscv/vm.c:27))

```c
void vcpu_arch_init(struct vcpu* vcpu, struct vm* vm)
{
    vcpu->arch.sbi_ctx.lock = SPINLOCK_INITVAL;
    vcpu->arch.sbi_ctx.state = vcpu->id == 0 ? STARTED : STOPPED;
}
```

#### VCPU 重置 ([`arch/riscv/vm.c`](hyper/bao-hypervisor/src/arch/riscv/vm.c:35))

```c
void vcpu_arch_reset(struct vcpu* vcpu, vaddr_t entry)
{
    memset(&vcpu->regs, 0, sizeof(struct arch_regs));
    
    csrs_sscratch_write((uintptr_t)&vcpu->regs);
    
    vcpu->regs.hstatus = HSTATUS_SPV | (1ULL << HSTATUS_VGEIN_OFF);
    
    if (DEFINED(RV64)) {
        vcpu->regs.hstatus |= HSTATUS_VSXL_64;
    }
    
    vcpu->regs.sstatus = SSTATUS_SPP_BIT | SSTATUS_FS_DIRTY | SSTATUS_XS_DIRTY;
    vcpu->regs.sepc = entry;
    vcpu->regs.a0 = vcpu->arch.hart_id = vcpu->id;
    vcpu->regs.a1 = 0;  // DTB 加载地址
    
    // 配置虚拟化 CSR
    csrs_hcounteren_write(HCOUNTEREN_TM);
    csrs_htimedelta_write(0);
    csrs_vsstatus_write(SSTATUS_SD | SSTATUS_FS_DIRTY | SSTATUS_XS_DIRTY);
    csrs_hie_write(0);
    csrs_vstvec_write(0);
    csrs_vsscratch_write(0);
    csrs_vsepc_write(0);
    csrs_vscause_write(0);
    csrs_vstval_write(0);
    csrs_hvip_write(0);
    csrs_vsatp_write(0);
}
```

### 异常处理

#### RISC-V 同步异常 ([`arch/riscv/sync_exceptions.c`](hyper/bao-hypervisor/src/arch/riscv/sync_exceptions.c:138))

```c
void sync_exception_handler(void)
{
    size_t pc_step = 0;
    unsigned long _scause = csrs_scause_read();
    
    // 检查是否来自虚拟机
    if (!(csrs_hstatus_read() & HSTATUS_SPV)) {
        internal_exception_handler(&cpu()->vcpu->regs.x[0]);
    }
    
    // 根据异常原因调用处理程序
    if (_scause < sync_handler_table_size && sync_handler_table[_scause]) {
        pc_step = sync_handler_table[_scause]();
    } else {
        ERROR("unkown synchronous exception (%d)", _scause);
    }
    
    vcpu_writepc(cpu()->vcpu, vcpu_readpc(cpu()->vcpu) + pc_step);
    if (vcpu_arch_is_on(cpu()->vcpu) && !cpu()->vcpu->active) {
        cpu_standby();
    }
}
```

**同步异常处理程序：**
- `SCAUSE_CODE_ECV`: 环境调用（SBI 调用）
- `SCAUSE_CODE_LGPF`: 加载 Guest 页错误
- `SCAUSE_CODE_SGPF`: 存储 Guest 页错误

#### Guest 页错误处理

```c
static size_t guest_page_fault_handler(void)
{
    vaddr_t addr = (csrs_htval_read() << 2) | (csrs_stval_read() & 0x3);
    
    // 查找模拟处理程序
    emul_handler_t handler = vm_emul_get_mem(cpu()->vcpu->vm, addr);
    if (handler != NULL) {
        unsigned long ins = csrs_htinst_read();
        size_t ins_size;
        
        // 如果 htinst 未提供信息，手动读取指令
        if (ins == 0) {
            vaddr_t ins_addr = csrs_sepc_read();
            ins = read_ins(ins_addr);
            ins_size = INS_SIZE(ins);
        } else if (is_pseudo_ins((uint32_t)ins)) {
            ERROR("fault on 1st stage page table walk");
        } else {
            ins_size = TINST_INS_SIZE(ins);
            ins = ins | 0x2;
        }
        
        // 解码加载/存储指令
        struct emul_access emul;
        if (!ins_ldst_decode(ins, &emul)) {
            ERROR("cant decode ld/st instruction");
        }
        emul.addr = addr;
        
        // 调用模拟处理程序
        if (handler(&emul)) {
            return ins_size;
        } else {
            ERROR("emulation handler failed (0x%x at 0x%x)", addr, csrs_sepc_read());
        }
    } else {
        ERROR("no emulation handler for abort(0x%x at 0x%x)", addr, csrs_sepc_read());
    }
}
```

---

## 平台支持

### 平台抽象层

Bao 使用平台抽象层来支持多种硬件平台。每个平台需要提供：

1. **平台描述**：内存区域、CPU 数量、设备信息
2. **驱动支持**：UART、GIC、SMMU 等
3. **启动代码**：平台特定的启动序列
4. **PSCI 支持**：电源管理接口

### 支持的平台

#### ARM 平台

| 平台 | 目录 | 状态 |
|------|------|------|
| FVP-A | `platform/fvp-a` | ✅ |
| FVP-R | `platform/fvp-r` | ✅ |
| HiKey 960 | `platform/hikey960` | ✅ |
| i.MX8M Plus | `platform/imx8mp-verdin` | ✅ |
| i.MX8QM | `platform/imx8qm` | ✅ |
| MPS3-AN536 | `platform/mps3-an536` | ✅ |
| QEMU AArch64 | `platform/qemu-aarch64-virt` | ✅ |
| Raspberry Pi 4 | `platform/rpi4` | ✅ |
| TX2 | `platform/tx2` | ✅ |
| Ultra96 | `platform/ultra96` | ✅ |
| ZCU102 | `platform/zcu102` | ✅ |

#### RISC-V 平台

| 平台 | 目录 | 状态 |
|------|------|------|
| QEMU RV32 | `platform/qemu-riscv32-virt` | ✅ |
| QEMU RV64 | `platform/qemu-riscv64-virt` | ✅ |

### 平台配置示例

每个平台提供一个 `*_desc.c` 文件，描述平台的硬件资源：

```c
struct platform platform = {
    .cpu_num = 4,
    .region_num = 2,
    .regions = (struct mem_region[]){
        {
            .base = 0x80000000,
            .size = 0x80000000,
        },
        {
            .base = 0x880000000,
            .size = 0x780000000,
        },
    },
    .arch = {
        .irqc = {
            .type = IRQC_GICV3,
            .gic_dist_addr = 0x08000000,
            .gic_redist_addr = 0x080A0000,
        },
    },
};
```

---

## 构建系统

### Makefile 架构 ([`Makefile`](hyper/bao-hypervisor/Makefile:1))

Bao 使用递归 Makefile 系统，支持多架构、多平台构建。

#### 构建变量

```makefile
DEBUG:=n                    # 调试模式
OPTIMIZATIONS:=2           # 优化级别
CONFIG=                    # 配置名称
PLATFORM=                  # 平台名称
CROSS_COMPILE=             # 交叉编译工具链前缀
```

#### 构建流程

```
1. 检查平台和配置
   ↓
2. 设置架构和工具链
   ↓
3. 包含架构和平台 Makefile
   ↓
4. 生成配置和平台定义头文件
   ↓
5. 编译源文件
   ↓
6. 链接生成 ELF
   ↓
7. 生成二进制文件
```

#### 关键生成文件

1. **汇编定义头文件** (`asm_defs.h`):
   - 从 `asm_defs.c` 生成汇编宏定义

2. **配置定义头文件** (`config_defs.h`):
   - 从配置 C 文件生成配置宏

3. **平台定义头文件** (`platform_defs.h`):
   - 从平台描述生成平台宏

#### 构建命令示例

```bash
# 构建 QEMU AArch64 平台
make PLATFORM=qemu-aarch64-virt CONFIG=example

# 构建 QEMU RISC-V64 平台
make PLATFORM=qemu-riscv64-virt CONFIG=example

# 调试构建
make DEBUG=y PLATFORM=qemu-aarch64-virt CONFIG=example

# 统计代码行数
make cloc PLATFORM=qemu-aarch64-virt CONFIG=example

# 清理构建
make clean
```

### 配置系统

#### 配置文件结构

配置文件位于 `configs/` 目录，每个配置包含：

```c
#include <bao.h>

struct config config = {
    .vmlist_size = 2,
    .vmlist = (struct vm_config[]){
        {
            .image = {
                .load_addr = 0x40080000,
                .base_addr = 0x40000000,
                .size = 0x800000,
            },
            .platform = {
                .cpu_num = 1,
                .region_num = 1,
                .regions = (struct vm_mem_region[]){
                    {
                        .base = 0x40000000,
                        .size = 0x10000000,
                    },
                },
            },
        },
    },
};
```

#### 配置元素

1. **VM 配置**：
   - 镜像加载地址和运行地址
   - CPU 数量和亲和性
   - 内存区域
   - 设备映射
   - 中断分配

2. **共享内存配置**：
   - 共享内存区域
   - IPC 通道

3. **Hypervisor 配置**：
   - 内存着色
   - 调试选项

---

## 关键特性与设计哲学

### 1. 静态分区

**优势：**
- 可预测的性能
- 无运行时调度开销
- 简化的资源管理
- 更小的 TCB

**实现：**
- 编译时确定所有资源分配
- 1:1 VCPU 到 PCPU 映射
- 静态内存分区
- 静态中断路由

### 2. 最小化 TCB

**设计原则：**
- 仅实现虚拟化必需的功能
- 无外部依赖
- 简化的代码路径
- 减少攻击面

**实现：**
- 约 10K-20K 行核心代码
- 无特权 VM（如 Linux）
- 直接硬件访问

### 3. 实时性保证

**关键机制：**
- 无调度器开销
- 确定性的中断延迟
- 静态资源分配
- 最小化虚拟化开销

**性能特性：**
- 中断透传：直接物理中断到虚拟中断映射
- I/O 透传：直接设备访问
- 无上下文切换：1:1 CPU 映射

### 4. 强隔离性

**隔离机制：**
- 两级地址转换
- 内存保护单元 (MMU/MPU)
- 中断隔离
- 设备隔离 (IOMMU)

**故障遏制：**
- VM 故障不影响其他 VM
- Hypervisor 故障导致系统重启
- 内存访问违规立即检测

### 5. 多架构支持

**架构抽象：**
- 统一的 VMM 接口
- 架构特定实现
- 平台抽象层

**支持特性：**
- ARMv8-A: 完整虚拟化支持
- ARMv8-R: MPU 支持
- RISC-V: H 扩展支持

### 6. 高级特性

#### 共享内存 ([`shmem.c`](hyper/bao-hypervisor/src/core/shmem.c))

- VM 间通信
- 高效数据共享
- 可配置大小和颜色

#### IPC ([`ipc.c`](hyper/bao-hypervisor/src/core/ipc.c))

- 基于共享内存的 IPC
- 中断通知
- 可配置的通道数量

#### 远程 I/O ([`remio.c`](hyper/bao-hypervisor/src/core/remio.c))

- 跨 VM 设备共享
- 前端/后端模型
- 模拟设备支持

#### IOMMU 支持

- SMMUv2 (ARM)
- 设备 DMA 隔离
- 地址转换

#### 内存着色

- 缓存分区
- 实时性优化
- 减少 Cache 争用

---

## 技术亮点

### 1. 轻量级设计

- **代码量小**：核心代码约 15K-20K 行
- **内存占用小**：Hypervisor 镜像通常 < 100KB
- **启动快速**：无复杂初始化流程

### 2. 高性能

- **零拷贝**：共享内存直接映射
- **中断透传**：无虚拟化开销
- **无调度**：1:1 CPU 映射

### 3. 可扩展性

- **模块化架构**：易于添加新平台
- **配置驱动**：灵活的 VM 配置
- **插件式驱动**：支持多种设备

### 4. 安全性

- **形式化验证友好**：简单的控制流
- **最小化攻击面**：小 TCB
- **强隔离**：硬件强制隔离

---

## 应用场景

### 1. 混合关键性系统

- 安全关键应用（RTOS）
- 非安全关键应用（Linux）
- 硬件隔离保证

### 2. 实时系统

- 确定性延迟
- 无调度抖动
- 实时性保证

### 3. 安全系统

- 安全隔离
- 故障遏制
- 可信执行环境

### 4. 嵌入式虚拟化

- 资源受限设备
- 多 OS 共存
- 设备虚拟化

---

## 与其他 Hypervisor 的比较

| 特性 | Bao | Jailhouse | Xen | KVM |
|------|-----|-----------|-----|-----|
| 类型 | Type-1 | Type-1 | Type-1 | Type-2 |
| 架构 | 静态分区 | 静态分区 | 动态调度 | 动态调度 |
| TCB | 极小 | 小 | 中等 | 大 |
| 调度器 | 无 | 无 | 有 | 有 |
| 实时性 | 优秀 | 优秀 | 中等 | 中等 |
| 复杂度 | 低 | 低 | 高 | 高 |
| 适用场景 | 嵌入式 | 嵌入式 | 服务器 | 桌面/服务器 |

---

## 总结

Bao Hypervisor 是一个专为现代多核嵌入式系统设计的轻量级静态分区化 hypervisor。其核心优势在于：

1. **极小的 TCB**：约 15K-20K 行代码，无外部依赖
2. **静态分区**：可预测的性能，无运行时调度开销
3. **强隔离性**：硬件强制隔离，故障遏制
4. **实时性保证**：确定性延迟，适合混合关键性系统
5. **多架构支持**：ARMv8-A、ARMv8-R、RISC-V

Bao 的设计哲学体现了"少即是多"的原则，通过最小化功能来实现最大的可靠性和性能。它特别适合需要强隔离性和实时性保证的嵌入式系统，如汽车电子、工业控制、航空航天等领域。

---

## 参考资料

1. **项目主页**: http://www.bao-project.org/
2. **源代码**: https://github.com/bao-project/bao-hypervisor.git
3. **文档**: https://bao-project.readthedocs.io/
4. **演示**: https://github.com/bao-project/bao-demos

### 学术论文

1. José Martins, Adriano Tavares, Marco Solieri, Marko Bertogna, and Sandro Pinto. "**Bao: A Lightweight Static Partitioning Hypervisor for Modern Multi-Core Embedded Systems**". NG-RES 2020.
2. José Martins and Sandro Pinto. "**Bao: a modern lightweight embedded hypervisor**". Embedded World Conference 2020.
3. José Martins and Sandro Pinto. "**Static Partitioning Virtualization on RISC-V**". RISC-V Summit 2020.
4. Bruno Sá, José Martins and Sandro Pinto. "**A First Look at RISC-V Virtualization from an Embedded Systems Perspective**". IEEE Transactions on Computers, 2021.
5. Samuel Pereira, João Sousa, Sandro Pinto, José Martins, David Cerdeira "**Bao-Enclave: Virtualization-based Enclaves for Arm**". 2022.
6. José Martins and Sandro Pinto. "**Shedding Light on Static Partitioning Hypervisors for Arm-based Mixed-Criticality Systems**". RTAS 2023.
7. José Martins and Sandro Pinto. "**Porting of a Static Partitioning Hypervisor to Arm's Cortex-R52**". Embedded Open Source Summit 2023.
8. David Cerdeira and José Martins. "**"Hello 'Bao' World" Tutorial**". Bao Half-Day Workshop 2023.
9. João Peixoto, José Martins, David Cerdeira and Sandro Pinto. "**BiRtIO: VirtIO for Real-Time Network Interface Sharing on the Bao Hypervisor**". IEEE Access, 2024.
10. Hidemasa Kawasaki and Soramichi Akiyama. "**Running Bao Hypervisor on gem5**". gem5 blog, 2024.

---

**报告生成时间**: 2026-01-08
**项目版本**: 基于当前代码库分析
**分析深度**: 深度代码级分析（5次迭代优化）

---

## 附录 A：ARMv8 架构深入分析（第一次迭代）

### A.1 GIC (Generic Interrupt Controller) 实现

#### A.1.1 GIC 核心架构 ([`gic.c`](hyper/bao-hypervisor/src/arch/armv8/gic.c:1))

Bao 实现了完整的 GIC 抽象层，支持 GICv2 和 GICv3 两个版本：

```c
// GIC 版本选择
#if (GIC_VERSION == GICV2)
#include <arch/gicv2.h>
#elif (GIC_VERSION == GICV3)
#include <arch/gicv3.h>
#endif
```

**核心数据结构：**
- `gicd`: GIC 分发器（Distributor）硬件接口
- `gicd_lock`: 保护 GICD 访问的自旋锁
- `gic_maintenance_id`: GIC 维护中断 ID

**初始化流程：**

1. **主核初始化 GICD**：
   - 清除所有中断的使能、挂起、激活状态
   - 设置所有中断为最低优先级
   - GICv2: 清除所有 CPU 目标
   - GICv3: 设置所有中断路由为无效
   - 启用分发器和亲和路由

2. **所有核心初始化 GICC**：
   - 设置优先级掩码
   - 启用 EOI 模式
   - 配置虚拟化控制

**中断处理流程：**

```c
void gic_handle()
{
    uint32_t ack = gicc_iar();  // 读取中断确认寄存器
    irqid_t id = bit32_extract(ack, GICC_IAR_ID_OFF, GICC_IAR_ID_LEN);
    
    if (id < GIC_FIRST_SPECIAL_INTID) {
        enum irq_res res = interrupts_handle(id);
        gicc_eoir(ack);  // 结束中断
        if (res == HANDLED_BY_HYP) {
            gicc_dir(ack);  // 丢弃中断
        }
    }
}
```

#### A.1.2 GICv2 特定实现 ([`gicv2.c`](hyper/bao-hypervisor/src/arch/armv8/gicv2.c:1))

**关键特性：**

1. **CPU 接口 (GICC)**：
   - PMR (优先级掩码寄存器)
   - CTLR (CPU 接口控制寄存器)
   - EOIR (中断结束寄存器)

2. **虚拟化接口 (GICH)**：
   - HCR (Hypervisor 控制寄存器)
   - LR (列表寄存器，用于虚拟中断)
   - VTR (虚拟化类型寄存器)

3. **SGI (软件生成中断)**：
   - 通过 SGIR 寄存器发送 IPI
   - 支持目标 CPU 位图

**状态保存/恢复：**

```c
void gicc_save_state(struct gicc_state* state)
{
    state->CTLR = gicc->CTLR;
    state->PMR = gicc->PMR;
    state->BPR = gicc->BPR;
    // 保存私有中断状态
    state->priv_ISENABLER = gicd->ISENABLER[0];
    state->priv_IPRIORITYR[i] = gicd->IPRIORITYR[i];
    // 保存虚拟化状态
    state->HCR = gich->HCR;
    for (size_t i = 0; i < gich_num_lrs(); i++) {
        state->LR[i] = gich->LR[i];
    }
}
```

**CPU 映射：**

```c
static cpuid_t gic_cpu_map[GIC_MAX_TARGETS];

// 将物理 CPU ID 映射到 GIC CPU ID
gic_cpu_map[cpu()->id] = (cpuid_t)gic_cpu_id;
```

#### A.1.3 GICv3 特定实现 ([`gicv3.c`](hyper/bao-hypervisor/src/arch/armv8/gicv3.c:1))

**关键改进：**

1. **重分发器 (GICR)**：
   - 每个 CPU 有独立的重分发器
   - 支持亲和路由
   - 更好的可扩展性

2. **系统寄存器访问**：
   - 使用 ICC_*_EL1 系统寄存器
   - 不需要 MMIO 访问
   - 更低的延迟

3. **亲和路由：**

```c
void gicd_set_route(irqid_t int_id, uint64_t route)
{
    if (gic_is_priv(int_id)) {
        return;
    }
    
    spin_lock(&gicd_lock);
    gicd->IROUTER[int_id] = route & GICD_IROUTER_AFF_MSK;
    spin_unlock(&gicd_lock);
}
```

**重分发器初始化：**

```c
static inline void gicr_init(void)
{
    // 唤醒重分发器
    gicr[cpu()->id].WAKER &= ~GICR_WAKER_ProcessorSleep_BIT;
    while (gicr[cpu()->id].WAKER & GICR_WAKER_ChildrenASleep_BIT) { }
    
    // 清除中断状态
    gicr[cpu()->id].IGROUPR0 = ~0U;
    gicr[cpu()->id].ICENABLER0 = ~0U;
    gicr[cpu()->id].ICPENDR0 = ~0U;
    gicr[cpu()->id].ICACTIVER0 = ~0U;
    
    // 设置优先级
    for (size_t i = 0; i < GIC_NUM_PRIO_REGS(GIC_CPU_PRIV); i++) {
        gicr[cpu()->id].IPRIORITYR[i] = ~0U;
    }
}
```

### A.2 虚拟 GIC (vGIC) 实现 ([`vgic.c`](hyper/bao-hypervisor/src/arch/armv8/vgic.c:1))

vGIC 是 Bao 中最复杂的组件之一，共 1239 行代码，实现了完整的中断虚拟化。

#### A.2.1 核心数据结构

**虚拟中断结构：**

```c
struct vgic_int {
    spinlock_t lock;           // 保护中断状态
    irqid_t id;               // 中断 ID
    uint8_t state;            // 状态 (INV, PEND, ACT, PENDACT)
    uint8_t prio;              // 优先级
    uint8_t cfg;               // 配置 (边沿/电平触发)
    bool enabled;              // 是否使能
    bool hw;                   // 是否为硬件中断
    bool in_lr;                // 是否在列表寄存器中
    uint8_t lr;                // LR 索引
    struct vcpu* owner;        // 拥有者 VCPU
    union {
        struct {
            uint8_t pend;   // 挂起位图
            uint8_t act;    // 激活 CPU
        } sgi;
        struct {
            cpuid_t redist;  // 重分发器 ID
        } phys;
    };
};
```

**vGIC 分发器：**

```c
struct vgicd {
    spinlock_t lock;
    uint32_t CTLR;              // 控制寄存器
    uint32_t TYPER;             // 类型寄存器
    uint32_t IIDR;              // 实现者 ID 寄存器
    size_t int_num;             // 中断数量
    struct vgic_int* interrupts; // 中断数组
    struct list spilled;          // 溢出中断列表
    spinlock_t spilled_lock;     // 溢出列表锁
};
```

#### A.2.2 列表寄存器 (LR) 管理

**LR 溢出策略：**

当虚拟中断数量超过硬件 LR 数量时，Bao 实现了智能溢出策略：

1. **优先级驱动溢出**：
   - 优先溢出最低优先级的待处理中断
   - 保留高优先级中断在 LR 中
   - 如果有多个待处理中断，优先溢出待处理中断

2. **LR 选择算法：**

```c
bool vgic_add_lr(struct vcpu* vcpu, struct vgic_int* interrupt)
{
    // 查找空闲 LR
    ssize_t lr_ind = -1;
    uint64_t elrsr = gich_get_elrsr();
    for (size_t i = 0; i < NUM_LRS; i++) {
        if (bit64_get(elrsr, i)) {
            lr_ind = (ssize_t)i;
            break;
        }
    }
    
    if (lr_ind < 0) {
        // 没有空闲 LR，需要溢出
        unsigned min_prio_pend = interrupt->prio, min_prio_act = interrupt->prio;
        unsigned min_id_act = interrupt->id, min_id_pend = interrupt->id;
        size_t pend_found = 0;
        ssize_t pend_ind = -1, act_ind = -1;
        
        // 查找最低优先级的 LR
        for (size_t i = 0; i < NUM_LRS; i++) {
            gic_lr_t lr = (gic_lr_t)gich_read_lr(i);
            irqid_t lr_id = (irqid_t)GICH_LR_VID(lr);
            unsigned lr_prio = (lr & GICH_LR_PRIO_MSK) >> GICH_LR_PRIO_OFF;
            gic_lr_t lr_state = (lr & GICH_LR_STATE_MSK);
            
            if (lr_state & GICH_LR_STATE_ACT) {
                if (lr_prio > min_prio_act ||
                    (lr_prio == min_prio_act && lr_id > min_id_act)) {
                    min_id_act = lr_id;
                    min_prio_act = lr_prio;
                    act_ind = (ssize_t)i;
                }
            } else if (lr_state & GICH_LR_STATE_PND) {
                if (lr_prio > min_prio_pend ||
                    (lr_prio == min_prio_pend && lr_id > min_id_pend)) {
                    min_id_pend = lr_id;
                    min_prio_pend = lr_prio;
                    pend_ind = (ssize_t)i;
                }
                pend_found++;
            }
        }
        
        if (pend_found > 1) {
            lr_ind = pend_ind;  // 多个待处理，选择待处理
        } else {
            lr_ind = act_ind;  // 否则选择激活
        }
        
        if (lr_ind >= 0) {
            vgic_spill_lr(vcpu, (size_t)lr_ind);  // 溢出选择的 LR
        }
    }
    
    if (lr_ind >= 0) {
        vgic_write_lr(vcpu, interrupt, (size_t)lr_ind);
        ret = true;
    } else {
        vgic_add_spilled(vcpu, interrupt);  // 添加到溢出列表
    }
    
    return ret;
}
```

#### A.2.3 中断所有权管理

**所有权获取：**

```c
bool vgic_get_ownership(struct vcpu* vcpu, struct vgic_int* interrupt)
{
    bool ret = false;
    
    if (interrupt->owner == vcpu) {
        ret = true;  // 已经拥有
    } else if (interrupt->owner == NULL) {
        interrupt->owner = vcpu;  // 获取所有权
        ret = true;
    }
    
    return ret;
}
```

**所有权让出：**

```c
void vgic_yield_ownership(struct vcpu* vcpu, struct vgic_int* interrupt)
{
    // 检查是否可以让出所有权
    if ((GIC_VERSION == GICV2 && gic_is_priv(interrupt->id)) ||
        !vgic_owns(vcpu, interrupt) ||
        interrupt->in_lr || (vgic_get_state(interrupt) & ACT)) {
        return;  // 不能让出
    }
    
    interrupt->owner = NULL;  // 让出所有权
}
```

#### A.2.4 中断路由和注入

**硬件中断注入：**

```c
void vgic_inject_hw(struct vcpu* vcpu, irqid_t id)
{
    struct vgic_int* interrupt = vgic_get_int(vcpu, id, vcpu->id);
    spin_lock(&interrupt->lock);
    interrupt->owner = vcpu;
    interrupt->state = PEND;
    interrupt->in_lr = false;
    vgic_add_lr(vcpu, interrupt);  // 添加到 LR
    spin_unlock(&interrupt->lock);
}
```

**软件中断 (SGI) 处理：**

GICv2 的 SGI 有特殊处理，因为它们是广播中断：

```c
void vgic_inject_sgi(struct vcpu* vcpu, struct vgic_int* interrupt, vcpuid_t source)
{
    // 设置挂起位图
    for (ssize_t i = GIC_MAX_TARGETS - 1; i >= 0; i--) {
        if (interrupt->sgi.pend & (1U << i)) {
            lr |= (((gic_lr_t)i) << GICH_LR_CPUID_OFF) & GICH_LR_CPUID_MSK;
            interrupt->sgi.pend &= (uint8_t)(~(1U << i));
            lr |= GICH_LR_STATE_PND;
            break;
        }
    }
    
    if (interrupt->sgi.pend) {
        lr |= GICH_LR_EOI_BIT;  // 需要再次 EOI
    }
}
```

#### A.2.5 GIC 维护中断

**维护中断处理：**

```c
void gic_maintenance_handler(irqid_t irq_id)
{
    uint32_t misr = gich_get_misr();
    
    if (misr & GICH_MISR_EOI) {
        vgic_handle_trapped_eoir(cpu()->vcpu);
    }
    
    if (misr & (GICH_MISR_NP | GICH_MISR_U)) {
        vgic_refill_lrs(cpu()->vcpu, !!(misr & GICH_MISR_NP));
    }
    
    if (misr & GICH_MISR_LRENP) {
        uint32_t hcr_el2 = gich_get_hcr();
        while (hcr_el2 & GICH_HCR_EOICount_MASK) {
            vgic_eoir_highest_spilled_active(cpu()->vcpu);
            hcr_el2 -= (1U << GICH_HCR_EOICount_OFF);
            gich_set_hcr(hcr_el2);
            hcr_el2 = gich_get_hcr();
        }
    }
}
```

**LR 重新填充：**

```c
static void vgic_refill_lrs(struct vcpu* vcpu, bool npie)
{
    uint64_t elrsr = gich_get_elrsr();
    ssize_t lr_ind = bit64_ffs(elrsr & BIT64_MASK(0, NUM_LRS));
    unsigned flags = npie ? PEND : ACT | PEND;
    
    spin_lock(&vcpu->vm->arch.vgic_spilled_lock);
    while (lr_ind >= 0) {
        struct list* list = NULL;
        struct vgic_int* irq = vgic_highest_prio_spilled(vcpu, flags, &list);
        
        if (irq != NULL) {
            spin_lock(&irq->lock);
            bool got_ownership = vgic_get_ownership(vcpu, irq);
            if (got_ownership) {
                list_rm(list, &irq->node);
                vgic_write_lr(vcpu, irq, (size_t)lr_ind);
            }
            spin_unlock(&irq->lock);
            if (!got_ownership) {
                continue;  // 所有权冲突，尝试下一个
            }
        } else {
            uint32_t hcr = gich_get_hcr();
            gich_set_hcr(hcr & ~(GICH_HCR_NPIE_BIT | GICH_HCR_UIE_BIT));
            break;  // 没有更多可填充的中断
        }
        
        flags = ACT | PEND;
        elrsr = gich_get_elrsr();
        lr_ind = bit64_ffs(elrsr & BIT64_MASK(0, NUM_LRS));
    }
    spin_unlock(&vcpu->vm->arch.vgic_spilled_lock);
}
```

### A.3 异常处理 ([`aborts.c`](hyper/bao-hypervisor/src/arch/armv8/aborts.c:1))

#### A.3.1 数据中止处理

**数据中止处理流程：**

```c
static void aborts_data_lower(unsigned long iss, unsigned long far, unsigned long il,
    unsigned long ec)
{
    // 检查是否有足够的信息
    if (!(iss & ESR_ISS_DA_ISV_BIT) || (iss & ESR_ISS_DA_FnV_BIT)) {
        ERROR("no information to handle data abort (0x%x)", far);
    }
    
    unsigned long DSFC = bit_extract(iss, ESR_ISS_DA_DSFC_OFF, ESR_ISS_DA_DSFC_LEN) & (0xf << 2);
    
    // 只处理转换故障和权限故障
    if (DSFC != ESR_ISS_DA_DSFC_TRNSLT && DSFC != ESR_ISS_DA_DSFC_PERMIS) {
        ERROR("data abort is not translation fault - cant deal with it");
    }
    
    vaddr_t addr = far;
    emul_handler_t handler = vm_emul_get_mem(cpu()->vcpu->vm, addr);
    
    if (handler != NULL) {
        struct emul_access emul;
        emul.addr = addr;
        emul.width = (1U << bit_extract(iss, ESR_ISS_DA_SAS_OFF, ESR_ISS_DA_SAS_LEN));
        emul.write = iss & ESR_ISS_DA_WnR_BIT ? true : false;
        emul.reg = bit_extract(iss, ESR_ISS_DA_SRT_OFF, ESR_ISS_DA_SRT_LEN);
        emul.reg_width = 4 + (4 * bit_extract(iss, ESR_ISS_DA_SF_OFF, ESR_ISS_DA_SF_LEN));
        emul.sign_ext = bit_extract(iss, ESR_ISS_DA_SSE_OFF, ESR_ISS_DA_SSE_LEN);
        
        // 调用模拟处理程序
        if (handler(&emul)) {
            unsigned long pc_step = 2 + (2 * il);
            vcpu_writepc(cpu()->vcpu, vcpu_readpc(cpu()->vcpu) + pc_step);
        } else {
            ERROR("data abort emulation failed (0x%x)", far);
        }
    } else {
        ERROR("no emulation handler for abort(0x%x at 0x%x)", addr, vcpu_readpc(cpu()->vcpu));
    }
}
```

#### A.3.2 系统调用处理

**HVC (Hypervisor Call) 处理：**

```c
static void hvc_handler(unsigned long iss, unsigned long far, unsigned long il, unsigned long ec)
{
    unsigned long fid = vcpu_readreg(cpu()->vcpu, 0);
    
    long ret = SMCC_E_NOT_SUPPORTED;
    switch (fid & ~SMCC_FID_FN_NUM_MSK) {
        case SMCC32_FID_STD_SRVC:
        case SMCC64_FID_STD_SRVC:
            ret = standard_service_call(fid);
            break;
        case SMCC32_FID_VND_HYP_SRVC:
        case SMCC64_FID_VND_HYP_SRVC:
            ret = hypercall(fid & SMCC_FID_FN_NUM_MSK);  // Hypercall
            break;
        default:
            WARNING("Unknown system call fid 0x%x", fid);
    }
    
    vcpu_writereg(cpu()->vcpu, 0, (unsigned long)ret);
}
```

**SMC (Secure Monitor Call) 处理：**

```c
static void smc_handler(unsigned long iss, unsigned long far, unsigned long il, unsigned long ec)
{
    unsigned long smc_fid = vcpu_readreg(cpu()->vcpu, 0);
    unsigned long x1 = vcpu_readreg(cpu()->vcpu, 1);
    unsigned long x2 = vcpu_readreg(cpu()->vcpu, 2);
    unsigned long x3 = vcpu_readreg(cpu()->vcpu, 3);
    
    if (is_psci_fid(smc_fid)) {
        ret = psci_smc_handler((uint32_t)smc_fid, x1, x2, x3);
    } else {
        INFO("unknown smc_fid 0x%lx", smc_fid);
    }
    
    // 调整 PC 到下一条指令
    vcpu_writepc(cpu()->vcpu, vcpu_readpc(cpu()->vcpu) + 4);
}
```

**系统寄存器访问处理：**

```c
static void sysreg_handler(unsigned long iss, unsigned long far, unsigned long il, unsigned long ec)
{
    regaddr_t reg_addr = UNDEFINED_REG_ADDR;
    if (ec == ESR_EC_RG_64) {
        reg_addr = reg_addr_translate(iss);
    } else {
        reg_addr = (iss & ESR_ISS_SYSREG_ADDR_32) | OP0_MRS_CP15;
    }
    
    emul_handler_t handler = vm_emul_get_reg(cpu()->vcpu->vm, reg_addr);
    if (handler != NULL) {
        struct emul_access emul;
        emul.addr = reg_addr;
        emul.width = 8;
        emul.write = iss & ESR_ISS_SYSREG_DIR ? false : true;
        emul.reg = bit_extract(iss, ESR_ISS_SYSREG_REG_OFF, ESR_ISS_SYSREG_REG_LEN);
        emul.reg_high = bit_extract(iss, ESR_ISS_SYSREG_REG2_OFF, ESR_ISS_SYSREG_REG2_LEN);
        emul.reg_width = 8;
        emul.multi_reg = (ec == ESR_EC_RG_64) ? true : false;
        emul.sign_ext = false;
        
        if (handler(&emul)) {
            unsigned long pc_step = 2 + (2 * il);
            vcpu_writepc(cpu()->vcpu, vcpu_readpc(cpu()->vcpu) + pc_step);
        } else {
            ERROR("register access emulation failed (0x%x)", reg_addr);
        }
    } else {
        ERROR("no emulation handler for register access (0x%x at 0x%x)", reg_addr,
            vcpu_readpc(cpu()->vcpu));
    }
}
```

### A.4 PSCI 电源管理 ([`psci.c`](hyper/bao-hypervisor/src/arch/armv8/psci.c:1))

#### A.4.1 PSCI 状态机

**VCPU 状态：**

```c
enum psci_state {
    OFF,           // CPU 关闭
    ON,            // CPU 运行
    ON_PENDING,    // CPU 正在启动
};
```

**CPU 挂起处理：**

```c
static int32_t psci_cpu_suspend_handler(uint32_t power_state, unsigned long entrypoint,
    unsigned long context_id)
{
    uint32_t state_type = power_state & PSCI_STATE_TYPE_BIT;
    int32_t ret;
    
    if (state_type) {
        // PSCI_STATE_TYPE_POWERDOWN:
        spin_lock(&cpu()->vcpu->arch.psci_ctx.lock);
        cpu()->vcpu->arch.psci_ctx.entrypoint = entrypoint;
        cpu()->vcpu->arch.psci_ctx.context_id = context_id;
        spin_unlock(&cpu()->vcpu->arch.psci_ctx.lock);
        ret = psci_power_down();
    } else {
        // PSCI_STATE_TYPE_STANDBY:
        ret = psci_standby();
    }
    
    return ret;
}
```

**CPU 关闭处理：**

```c
static int32_t psci_cpu_off_handler(void)
{
    // 设置状态为 OFF
    spin_lock(&cpu()->vcpu->arch.psci_ctx.lock);
    cpu()->vcpu->arch.psci_ctx.state = OFF;
    spin_unlock(&cpu()->vcpu->arch.psci_ctx.lock);
    
    cpu_powerdown();
    
    // 立即设置为 ON（用于下次启动）
    spin_lock(&cpu()->vcpu->arch.psci_ctx.lock);
    cpu()->vcpu->arch.psci_ctx.state = ON;
    spin_unlock(&cpu()->vcpu->arch.psci_ctx.lock);
    
    return PSCI_E_DENIED;
}
```

**CPU 启动处理：**

```c
static int32_t psci_cpu_on_handler(unsigned long target_cpu, unsigned long entrypoint,
    unsigned long context_id)
{
    int32_t ret;
    struct vm* vm = cpu()->vcpu->vm;
    struct vcpu* target_vcpu = vm_get_vcpu_by_mpidr(vm, target_cpu);
    
    if (target_vcpu != NULL) {
        bool already_on = true;
        spin_lock(&cpu()->vcpu->arch.psci_ctx.lock);
        if (target_vcpu->arch.psci_ctx.state == OFF) {
            target_vcpu->arch.psci_ctx.state = ON_PENDING;
            target_vcpu->arch.psci_ctx.entrypoint = entrypoint;
            target_vcpu->arch.psci_ctx.context_id = context_id;
            fence_sync_write();
            already_on = false;
        }
        spin_unlock(&cpu()->vcpu->arch.psci_ctx.lock);
        
        if (already_on) {
            return PSCI_E_ALREADY_ON;
        }
        
        // 发送 IPI 唤醒目标 CPU
        cpuid_t pcpuid = vm_translate_to_pcpuid(vm, target_vcpu->id);
        if (pcpuid == INVALID_CPUID) {
            ret = PSCI_E_INVALID_PARAMS;
        } else {
            struct cpu_msg msg = { (uint32_t)PSCI_CPUMSG_ID, PSCI_MSG_ON, 0 };
            cpu_send_msg(pcpuid, &msg);
            ret = PSCI_E_SUCCESS;
        }
    } else {
        ret = PSCI_E_INVALID_PARAMS;
    }
    
    return ret;
}
```

**唤醒处理：**

```c
void psci_wake_from_off(void)
{
    if (cpu()->vcpu == NULL) {
        return;
    }
    
    spin_lock(&cpu()->vcpu->arch.psci_ctx.lock);
    if (cpu()->vcpu->arch.psci_ctx.state == ON_PENDING) {
        vcpu_arch_reset(cpu()->vcpu, cpu()->vcpu->arch.psci_ctx.entrypoint);
        cpu()->vcpu->arch.psci_ctx.state = ON;
        vcpu_writereg(cpu()->vcpu, 0, cpu()->vcpu->arch.psci_ctx.context_id);
    }
    spin_unlock(&cpu()->vcpu->arch.psci_ctx.lock);
}
```

### A.5 SMMUv2 IOMMU 实现 ([`smmuv2.c`](hyper/bao-hypervisor/src/arch/armv8/armv8-a/smmuv2.c:1))

#### A.5.1 SMMUv2 架构

**SMMUv2 特性检查：**

```c
static void smmu_check_features(void)
{
    unsigned version = bit32_extract(smmu.hw.glbl_rs0->IDR7,
        SMMUV2_IDR7_MAJOR_OFF, SMMUV2_IDR7_MAJOR_LEN);
    if (version != 2) {
        ERROR("smmu unsupported version: %d", version);
    }
    
    // 检查必需的特性
    if (!(smmu.hw.glbl_rs0->IDR0 & SMMUV2_IDR0_S2TS_BIT)) {
        ERROR("smmuv2 does not support 2nd stage translation");
    }
    
    if (!(smmu.hw.glbl_rs0->IDR0 & SMMUV2_IDR0_SMS_BIT)) {
        ERROR("smmuv2 does not support stream match");
    }
    
    if (!(smmu.hw.glbl_rs0->IDR0 & SMMUV2_IDR0_BTM_BIT)) {
        ERROR("smmuv2 does not support tlb maintenance broadcast");
    }
    
    if (!(smmu.hw.glbl_rs0->IDR2 & SMMUV2_IDR2_PTFSv8_4kB_BIT)) {
        ERROR("smmuv2 does not support 4kb page granule");
    }
}
```

#### A.5.2 上下文管理

**上下文分配：**

```c
ssize_t smmu_alloc_ctxbnk(void)
{
    spin_lock(&smmu.ctx_lock);
    // 查找空闲的上下文块
    ssize_t nth = bitmap_find_nth(smmu.ctxbank_bitmap, smmu.ctx_num, 1, 0, BITMAP_NOT_SET);
    if (nth >= 0) {
        bitmap_set(smmu.ctxbank_bitmap, (size_t)nth);
    }
    spin_unlock(&smmu.ctx_lock);
    
    return nth;
}
```

**上下文写入：**

```c
void smmu_write_ctxbnk(size_t ctx_id, paddr_t root_pt, asid_t vm_id)
{
    spin_lock(&smmu.ctx_lock);
    if (!bitmap_get(smmu.ctxbank_bitmap, ctx_id)) {
        ERROR("smmu ctx %d is already allocated", ctx_id);
    } else {
        // 设置类型为 Stage-2
        smmu.hw.glbl_rs1->CBAR[ctx_id] = SMMUV2_CBAR_VMID(vm_id);
        smmu.hw.glbl_rs1->CBA2R[ctx_id] = SMMUV2_CBAR_VA64;
        
        // 配置 TCR
        uint32_t tcr = ((parange << SMMUV2_TCR_PS_OFF) & SMMUV2_TCR_PS_MSK);
        size_t t0sz = 64 - parange_table[parange];
        tcr |= SMMUV2_TCR_TG0_4K;
        tcr |= SMMUV2_TCR_ORGN0_WB_RA_WA;
        tcr |= SMMUV2_TCR_IRGN0_WB_RA_WA;
        tcr |= SMMUV2_TCR_T0SZ(t0sz);
        tcr |= SMMUV2_TCR_SH0_IS;
        tcr |= ((parange_table[parange] < 44) ?
                 SMMUV2_TCR_SL0_1 : SMMUV2_TCR_SL0_0);
        smmu.hw.cntxt[ctx_id].TCR = tcr;
        
        // 设置页表基址
        smmu.hw.cntxt[ctx_id].TTBR0 = root_pt &
            SMMUV2_CB_TTBA(smmu_cb_ttba_offset(t0sz));
        
        // 配置 SCTLR
        uint32_t sctlr = smmu.hw.cntxt[ctx_id].SCTLR;
        sctlr = SMMUV2_SCTLR_CLEAR(sctlr);
        sctlr |= SMMUV2_SCTLR_DEFAULT;
        smmu.hw.cntxt[ctx_id].SCTLR |= sctlr;
    }
    spin_unlock(&smmu.ctx_lock);
}
```

#### A.5.3 流匹配条目 (SME) 管理

**SME 兼容性检查：**

```c
bool smmu_compatible_sme_exists(streamid_t mask, streamid_t id, size_t ctx, bool group)
{
    bool included = false;
    size_t sme = 0;
    
    spin_lock(&smmu.sme_lock);
    smmu_for_each_sme(sme) {
        streamid_t sme_mask = smmu_sme_get_mask(sme);
        streamid_t mask_r = mask & sme_mask;
        streamid_t diff_id = (smmu_sme_get_id(sme) ^ id) & ~(mask | sme_mask);
        
        if (!diff_id) {
            // 只有组到组或设备到组可以合并
            if (((group || smmu_sme_is_group(sme)) &&
                 (mask_r == mask || mask_r == sme_mask)) &&
                ctx == smmu_sme_get_ctx(sme)) {
                // 兼容的条目
                if (mask > sme_mask) {
                    bitmap_clear(smmu.sme_bitmap, sme);
                } else {
                    included = true;
                    break;
                }
            } else {
                ERROR("SMMU sme conflict");
            }
        }
    }
    spin_unlock(&smmu.sme_lock);
    
    return included;
}
```

**SME 写入：**

```c
void smmu_write_sme(size_t sme, streamid_t mask, streamid_t id, bool group)
{
    spin_lock(&smmu.sme_lock);
    if (!bitmap_get(smmu.sme_bitmap, sme)) {
        ERROR("smmu: trying to write unallocated sme %d", sme);
    } else {
        smmu.hw.glbl_rs0->SMR[sme] = mask << SMMU_SMR_MASK_OFF;
        smmu.hw.glbl_rs0->SMR[sme] |= id & SMMU_ID_MSK;
        smmu.hw.glbl_rs0->SMR[sme] |= SMMUV2_SMR_VALID;
        
        if (group) {
            bitmap_set(smmu.grp_bitmap, sme);
        }
    }
    spin_unlock(&smmu.sme_lock);
}
```

**S2C (Stage-2 Context) 绑定：**

```c
void smmu_write_s2c(size_t sme, size_t ctx_id)
{
    spin_lock(&smmu.sme_lock);
    if (!bitmap_get(smmu.ctxbank_bitmap, ctx_id)) {
        ERROR("smmu: trying to write unallocated s2c %d", ctx_id);
    } else if (!bitmap_get(smmu.sme_bitmap, sme)) {
        ERROR("smmu: trying to bind unallocated sme %d", sme);
    } else {
        // 初始上下文是转换上下文
        uint32_t s2cr = smmu.hw.glbl_rs0->S2CR[sme];
        
        s2cr = S2CR_CLEAR(s2cr);
        s2cr |= S2CR_DFLT;
        s2cr |= ctx_id & S2CR_CBNDX_MASK;
        
        smmu.hw.glbl_rs0->S2CR[sme] = s2cr;
    }
    spin_unlock(&smmu.sme_lock);
}
```

### A.6 页表管理 ([`page_table.c`](hyper/bao-hypervisor/src/arch/armv8/armv8-a/page_table.c:1))

#### A.6.1 页表描述符

**AArch64 页表描述符：**

```c
struct page_table_dscr armv8_pt_dscr = {
    .lvls = 4,                              // 4 级页表
    .lvl_wdt = (size_t[]){ 48, 39, 30, 21 },  // 每级宽度
    .lvl_off = (size_t[]){ 39, 30, 21, 12 },  // 每级偏移
    .lvl_term = (bool[]){ false, true, true, true },  // 终止标志
};
```

**Stage-2 页表描述符：**

```c
struct page_table_dscr armv8_pt_s2_dscr = {
    .lvls = 4,                              // 4 级页表
    .lvl_wdt = (size_t[]){ 48, 39, 30, 21 },  // 每级宽度
    .lvl_off = (size_t[]){ 39, 30, 21, 12 },  // 每级偏移
    .lvl_term = (bool[]){ false, true, true, true },  // 终止标志
};
```

**物理地址范围表：**

```c
size_t parange_table[] = { 32, 36, 40, 42, 44, 48 };
```

#### A.6.2 页表操作

**递归页表设置：**

```c
void pt_set_recursive(struct page_table* pt, size_t index)
{
    paddr_t pa;
    mem_translate(&cpu()->as, (vaddr_t)pt->root, &pa);
    pte_t* pte = cpu()->as.pt.root + index;
    pte_set(pte, pa, PTE_TABLE, PTE_HYP_FLAGS);
    pt->arch.rec_ind = index;
    pt->arch.rec_mask = 0;
    
    // 更新递归掩码
    size_t cpu_rec_ind = cpu()->as.pt.arch.rec_ind;
    for (size_t i = 0; i < pt->dscr->lvls; i++) {
        size_t lvl_off = pt->dscr->lvl_off[i];
        pt->arch.rec_mask |= cpu_rec_ind << lvl_off;
    }
}
```

**页表项获取：**

```c
pte_t* pt_get_pte(struct page_table* pt, size_t lvl, vaddr_t va)
{
    struct page_table* cpu_pt = &cpu()->as.pt;
    
    size_t rec_ind_off = cpu_pt->dscr->lvl_off[cpu_pt->dscr->lvls - lvl - 1];
    size_t rec_ind_len = cpu_pt->dscr->lvl_wdt[cpu_pt->dscr->lvls - lvl - 1];
    uintptr_t rec_ind_mask = (uintptr_t)PTE_MASK(rec_ind_off, rec_ind_len - rec_ind_off);
    uintptr_t addr = (uintptr_t)(cpu_pt->arch.rec_mask & ~PTE_MASK(0, rec_ind_len));
    addr |= (pt->arch.rec_ind << rec_ind_off) & rec_ind_mask;
    addr |= (uintptr_t)((va >> pt->dscr->lvl_off[lvl]) * sizeof(pte_t) & PTE_MASK(0, rec_ind_off));
    
    return (pte_t*)addr;
}
```

**页表项类型检查：**

```c
bool pte_page(struct page_table* pt, pte_t* pte, size_t lvl)
{
    if (!pt_lvl_terminal(pt, lvl)) {
        return false;
    }
    
    if (lvl != pt->dscr->lvls - 1) {
        return (*pte & PTE_TYPE_MSK) == PTE_SUPERPAGE;
    }
    
    return (*pte & PTE_TYPE_MSK) == PTE_PAGE;
}

bool pte_table(struct page_table* pt, pte_t* pte, size_t lvl)
{
    if (lvl == pt->dscr->lvls - 1) {
        return false;
    }
    
    return (*pte & PTE_TYPE_MSK) == PTE_TABLE;
}
```

### A.7 ARMv8 虚拟化配置 ([`vmm.c`](hyper/bao-hypervisor/src/arch/armv8/vmm.c:1))

#### A.7.1 HCR_EL2 配置

```c
void vmm_arch_init()
{
    vmm_arch_profile_init();
    
    uint64_t hcr = HCR_VM_BIT | HCR_IMO_BIT | HCR_FMO_BIT | HCR_TSC_BIT;
    
    if (DEFINED(AARCH64)) {
        hcr |= HCR_RW_BIT | HCR_APK_BIT | HCR_API_BIT;
    }
    
    sysreg_hcr_el2_write(hcr);  // 配置 HCR_EL2
    sysreg_cptr_el2_write(0);    // 配置 CPTR_EL2
}
```

**HCR_EL2 位详细说明：**

| 位 | 名称 | 功能 |
|-----|------|------|
| VM | Stage-2 转换使能 | 启用 Stage-2 地址转换 |
| IMO | IRQ 路由到 EL2 | 将所有 IRQ 路由到 EL2 |
| FMO | FIQ 路由到 EL2 | 将所有 FIQ 路由到 EL2 |
| TSC | 访问计数器陷阱 | 陷阱访问计数器指令 |
| RW | EL1 执行状态 | 0=AArch32, 1=AArch64 |
| APK | 指针认证陷阱 | 陷阱 EL0 的指针认证指令 |
| API | 指针认证陷阱 | 陷阱 EL1 的指针认证指令 |

**CPTR_EL2 配置：**

```c
sysreg_cptr_el2_write(0);  // 允许访问所有系统寄存器
```

### A.8 ARMv8 架构总结

**关键技术特性：**

1. **完整的 GIC 虚拟化**：
   - 支持 GICv2 和 GICv3
   - 智能的 LR 溢出策略
   - 中断所有权管理
   - 维护中断处理

2. **全面的异常处理**：
   - 数据中止模拟
   - 系统调用处理 (HVC/SMC)
   - 系统寄存器访问模拟

3. **PSCI 电源管理**：
   - CPU 启动/关闭/挂起
   - 状态机管理
   - IPI 唤醒机制

4. **SMMUv2 IOMMU 支持**：
   - Stage-2 地址转换
   - 流匹配条目管理
   - 上下文管理
   - 设备 DMA 隔离

5. **灵活的页表管理**：
   - 4 级页表遍历
   - 递归页表支持
   - Stage-1 和 Stage-2 转换

**设计优势：**

- **性能优化**：智能的 LR 溢出策略减少中断延迟
- **可扩展性**：支持大量虚拟中断
- **强隔离**：硬件强制的中断和内存隔离
- **实时性**：确定性的中断处理路径
- **灵活性**：支持多种 GIC 版本和平台

---

## 附录 B：RISC-V 架构深入分析（第二次迭代）

### B.1 RISC-V 虚拟化实现 ([`vm.c`](hyper/bao-hypervisor/src/arch/riscv/vm.c:1))

#### B.1.1 VM 架构初始化

```c
void vm_arch_init(struct vm* vm, const struct vm_config* vm_config)
{
    paddr_t root_pt_pa;
    mem_translate(&cpu()->as, (vaddr_t)vm->as.pt.root, &root_pt_pa);
    
    // 配置 hgatp (Guest Address Translation and Protection)
    unsigned long hgatp = (root_pt_pa >> PAGE_SHIFT) |
                         (HGATP_MODE_DFLT) |
                         ((vm->id << HGATP_VMID_OFF) & HGATP_VMID_MSK);
    
    csrs_hgatp_write(hgatp);
    
    virqc_init(vm, &vm_config->platform.arch.irqc);
}
```

**HGATP 寄存器配置：**
- `hgatp[63:60]`: VMID（虚拟机 ID）
- `hgatp[59:44]`: MODE（页表模式，默认 Sv39x4）
- `hgatp[43:0]`: PPN（根页表物理页号）

#### B.1.2 VCPU 架构初始化

```c
void vcpu_arch_init(struct vcpu* vcpu, struct vm* vm)
{
    UNUSED_ARG(vm);
    
    vcpu->arch.sbi_ctx.lock = SPINLOCK_INITVAL;
    vcpu->arch.sbi_ctx.state = vcpu->id == 0 ? STARTED : STOPPED;
}
```

**SBI 上下文状态机：**
- `STARTED`: VCPU 正在运行
- `STOPPED`: VCPU 已停止
- `START_PENDING`: VCPU 正在启动过程中

#### B.1.3 VCPU 重置

```c
void vcpu_arch_reset(struct vcpu* vcpu, vaddr_t entry)
{
    memset(&vcpu->regs, 0, sizeof(struct arch_regs));
    
    csrs_sscratch_write((uintptr_t)&vcpu->regs);
    
    // 设置 HSTATUS 寄存器
    vcpu->regs.hstatus = HSTATUS_SPV | (1ULL << HSTATUS_VGEIN_OFF);
    
    if (DEFINED(RV64)) {
        vcpu->regs.hstatus |= HSTATUS_VSXL_64;
    }
    
    // 设置 SSTATUS 寄存器
    vcpu->regs.sstatus = SSTATUS_SPP_BIT | SSTATUS_FS_DIRTY | SSTATUS_XS_DIRTY;
    vcpu->regs.sepc = entry;
    vcpu->regs.a0 = vcpu->arch.hart_id = vcpu->id;
    vcpu->regs.a1 = 0;  // DTB 加载地址
    
    // 配置虚拟化 CSR
    csrs_hcounteren_write(HCOUNTEREN_TM);
    csrs_htimedelta_write(0);
    csrs_vsstatus_write(SSTATUS_SD | SSTATUS_FS_DIRTY | SSTATUS_XS_DIRTY);
    csrs_hie_write(0);
    csrs_vstvec_write(0);
    csrs_vsscratch_write(0);
    csrs_vsepc_write(0);
    csrs_vscause_write(0);
    csrs_vstval_write(0);
    csrs_hvip_write(0);
    csrs_vsatp_write(0);
}
```

**CSR 配置说明：**

| CSR | 功能 |
|-----|------|
| `hcounteren` | 计数器使能（时间戳） |
| `htimedelta` | 时间偏移 |
| `vsstatus` | 虚拟机状态寄存器 |
| `hie` | 中断使能 |
| `vstvec` | 中断向量表地址 |
| `vsscratch` | 临时寄存器 |
| `vsepc` | 异常 PC |
| `vscause` | 异常原因 |
| `vstval` | 异常值 |
| `hvip` | 虚拟中断挂起 |
| `vsatp` | 虚拟机地址转换 |

#### B.1.4 寄存器访问

```c
unsigned long vcpu_readreg(struct vcpu* vcpu, unsigned long reg)
{
    if ((reg <= 0) || (reg > 31)) {
        return 0;
    }
    return vcpu->regs.x[reg - 1];
}

void vcpu_writereg(struct vcpu* vcpu, unsigned long reg, unsigned long val)
{
    if ((reg <= 0) || (reg > 31)) {
        return;
    }
    vcpu->regs.x[reg - 1] = val;
}

unsigned long vcpu_readpc(struct vcpu* vcpu)
{
    return vcpu->regs.sepc;
}

void vcpu_writepc(struct vcpu* vcpu, unsigned long pc)
{
    vcpu->regs.sepc = pc;
}

bool vcpu_arch_is_on(struct vcpu* vcpu)
{
    return vcpu->arch.sbi_ctx.state == STARTED;
}
```

### B.2 RISC-V VMM 初始化 ([`vmm.c`](hyper/bao-hypervisor/src/arch/riscv/vmm.c:1))

#### B.2.1 异常委托配置

```c
void vmm_arch_init()
{
    // 委托所有不由 hypervisor 处理的中断和异常
    csrs_hideleg_write(HIDELEG_VSSI | HIDELEG_VSTI | HIDELEG_VSEI);
    csrs_hedeleg_write(HEDELEG_ECU | HEDELEG_IPF | HEDELEG_LPF | HEDELEG_SPF);
    
    // 检查 Sstc 扩展
    if (CPU_HAS_EXTENSION(CPU_EXT_SSTC)) {
        csrs_henvcfg_set(HENVCFG_STCE);
        bool sstc_present = (csrs_henvcfg_read() & HENVCFG_STCE) != 0;
        if (cpu_is_master() && !sstc_present) {
            ERROR("Platform configured to use Sstc extension, but extension not present.");
        }
        csrs_stimecmp_write(~0U);
    } else {
        csrs_henvcfg_clear(HENVCFG_STCE);
    }
}
```

**委托异常类型：**

| 异常类型 | 委托目标 | 说明 |
|---------|---------|------|
| `VSSI` | Supervisor Software Interrupt | 软件中断 |
| `VSTI` | Supervisor Timer Interrupt | 定时器中断 |
| `VSEI` | Supervisor External Interrupt | 外部中断 |
| `ECU` | Environment Call from U-mode | 用户态环境调用 |
| `IPF` | Instruction Page Fault | 指令页错误 |
| `LPF` | Load Page Fault | 加载页错误 |
| `SPF` | Store Page Fault | 存储页错误 |

### B.3 SBI (Supervisor Binary Interface) 实现 ([`sbi.c`](hyper/bao-hypervisor/src/arch/riscv/sbi.c:1))

#### B.3.1 SBI 扩展 ID 定义

```c
#define SBI_EXTID_BASE                  (0x10)
#define SBI_EXTID_TIME                  (0x54494D45)
#define SBI_EXTID_IPI                   (0x735049)
#define SBI_EXTID_RFNC                  (0x52464E43)
#define SBI_EXTID_HSM                   (0x48534D)
#define SBI_EXTID_BAO                   (0x08000ba0)
```

#### B.3.2 SBI 函数 ID 定义

**基础扩展 (0x10)：**
- `SBI_GET_SBI_SPEC_VERSION_FID`: 获取 SBI 规范版本
- `SBI_GET_SBI_IMPL_ID_FID`: 获取实现 ID
- `SBI_GET_SBI_IMPL_VERSION_FID`: 获取实现版本
- `SBI_PROBE_EXTENSION_FID`: 探测扩展支持

**时间扩展 (0x54494D45)：**
- `SBI_SET_TIMER_FID`: 设置定时器

**IPI 扩展 (0x735049)：**
- `SBI_SEND_IPI_FID`: 发送 IPI

**RFNC 扩展 (0x52464E43)：**
- `SBI_REMOTE_FENCE_I_FID`: 远程内存屏障
- `SBI_REMOTE_SFENCE_VMA_FID`: 虚拟内存屏障
- `SBI_REMOTE_HFENCE_GVMA_FID`: 虚拟机内存屏障

**HSM 扩展 (0x48534D)：**
- `SBI_HART_START_FID`: 启动 Hart
- `SBI_HART_STOP_FID`: 停止 Hart
- `SBI_HART_STATUS_FID`: 获取 Hart 状态
- `SBI_HART_SUSPEND_FID`: 挂起 Hart

**Bao 扩展 (0x08000ba0)：**
- Bao 特定的 hypercall 接口

#### B.3.3 SBI ECall 实现

```c
static inline struct sbiret sbi_ecall(unsigned long eid, unsigned long fid,
    unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
    unsigned long a4, unsigned long a5)
{
    register unsigned long _a0 __asm__("a0") = a0;
    register unsigned long _a1 __asm__("a1") = a1;
    register unsigned long _a2 __asm__("a2") = a2;
    register unsigned long _a3 __asm__("a3") = a3;
    register unsigned long _a4 __asm__("a4") = a4;
    register unsigned long _a5 __asm__("a5") = a5;
    register unsigned long _a6 __asm__("a6") = fid;
    register unsigned long _a7 __asm__("a7") = eid;
    
    __asm__ volatile("ecall" : "+r"(_a0), "+r"(_a1), "r"(_a2), "r"(_a3), "r"(_a4),
                      "r"(_a5), "r"(_a6), "r"(_a7) : "memory");
    
    struct sbiret ret = { .error = (long)_a0, .value = (long)_a1 };
    
    return ret;
}
```

**ECall 调用约定：**
- `a0`: 返回值（错误码）
- `a1`: 返回值（函数结果）
- `a2-a6`: 函数参数
- `a6`: 扩展 ID (EID)
- `a7`: 函数 ID (FID)

---

## 附录 C：内存保护和虚拟化机制（第三次迭代）

### C.1 MMU 内存保护 ([`mmu/mem.c`](hyper/bao-hypervisor/src/core/mmu/mem.c:1))

#### C.1.1 地址空间段管理

**段结构定义：**

```c
struct section {
    vaddr_t beg;              // 段起始地址
    vaddr_t end;              // 段结束地址
    bool shared;               // 是否共享
    spinlock_t lock;           // 段锁
};
```

**Hypervisor 地址空间段：**

```c
struct section hyp_secs[] = {
    [SEC_HYP_GLOBAL] = {
        (vaddr_t)&_dmem_beg,
        (vaddr_t)&_cpu_private_beg - 1,
        MEM_SEC_SHARED,
        SPINLOCK_INITVAL
    },
    [SEC_HYP_IMAGE] = {
        (vaddr_t)&_image_start,
        (vaddr_t)&_image_end - 1,
        MEM_SEC_SHARED,
        SPINLOCK_INITVAL
    },
    [SEC_HYP_PRIVATE] = {
        (vaddr_t)&_cpu_private_beg,
        (vaddr_t)&_cpu_private_end - 1,
        MEM_SEC_NOT_SHARED,
        SPINLOCK_INITVAL
    },
    [SEC_HYP_VM] = {
        (vaddr_t)&_vm_beg,
        (vaddr_t)&_vm_end - 1,
        MEM_SEC_SHARED,
        SPINLOCK_INITVAL
    },
};
```

**段类型说明：**

| 段类型 | 地址范围 | 共享性 | 用途 |
|---------|---------|--------|------|
| `SEC_HYP_GLOBAL` | 全局数据 | 共享 | Hypervisor 全局变量 |
| `SEC_HYP_IMAGE` | Hypervisor 镜像 | 共享 | Hypervisor 代码和数据 |
| `SEC_HYP_PRIVATE` | CPU 私有 | 不共享 | CPU 私有栈和状态 |
| `SEC_HYP_VM` | VM 区域 | 共享 | VM 映射区域 |

#### C.1.2 页池管理

**页池着色分配：**

```c
bool pp_alloc_clr(struct page_pool* pool, size_t n, colormap_t colors, struct ppages* ppages)
{
    size_t allocated = 0;
    size_t first_index = 0;
    bool ok = false;

    ppages->colors = colors;
    ppages->num_pages = 0;

    spin_lock(&pool->lock);

    // 从上次已知空闲位置开始搜索
    size_t index = pp_next_clr(pool->base, pool->last, colors);
    size_t top = pool->num_pages;

    // 两次迭代：从上次位置到顶部，再从开始到上次位置
    for (size_t i = 0; i < 2 && !ok; i++) {
        while ((allocated < n) && (index < top)) {
            allocated = 0;

            // 查找目标颜色的第一个空闲页
            while ((index < top) && bitmap_get(pool->bitmap, index)) {
                index = pp_next_clr(pool->base, ++index, colors);
            }
            first_index = index;

            // 计算连续的空闲页数
            while ((index < top) && (bitmap_get(pool->bitmap, index) == 0) && (allocated < n)) {
                allocated++;
                index = pp_next_clr(pool->base, ++index, colors);
            }
        }

        if (allocated == n) {
            // 找到匹配颜色模式的连续页
            ppages->num_pages = n;
            ppages->base = pool->base + (first_index * PAGE_SIZE);
            for (size_t j = 0; j < n; j++) {
                first_index = pp_next_clr(pool->base, first_index, colors);
                bitmap_set(pool->bitmap, first_index++);
            }
            pool->free -= n;
            pool->last = first_index;
            ok = true;
            break;
        } else {
            // 第一次迭代失败，从开始重新搜索
            index = pp_next_clr(pool->base, 0, colors);
        }
    }

    spin_unlock(&pool->lock);
    return ok;
}
```

**着色分配算法：**
1. **颜色模式匹配**：只分配符合指定颜色模式的物理页
2. **连续性保证**：确保分配的页在物理内存中连续
3. **双向搜索**：从上次位置和开始位置两次搜索以提高效率
4. **位图管理**：使用位图跟踪页分配状态

#### C.1.3 页表操作

**页表分配：**

```c
static inline pte_t* mem_alloc_pt(struct addr_space* as, pte_t* parent, size_t lvl, vaddr_t addr)
{
    size_t ptsize = NUM_PAGES(pt_size(&as->pt, lvl + 1));
    struct ppages ppage = mem_alloc_ppages(as->colors, ptsize,
        ptsize > 1 ? MEM_ALIGN_REQ : MEM_ALIGN_NOT_REQ);
    if (ppage.num_pages == 0) {
        return NULL;
    }
    pte_t pte_dflt_val = PTE_INVALID | (*parent & PTE_RSW_MSK);
    pte_set(parent, ppage.base, PTE_TABLE, PTE_HYP_FLAGS);
    fence_sync_write();
    pte_t* temp_pt = pt_get(&as->pt, lvl + 1, addr);
    for (size_t i = 0; i < pt_nentries(&as->pt, lvl + 1); i++) {
        temp_pt[i] = pte_dflt_val;
    }
    return temp_pt;
}
```

**页表膨胀：**

```c
static void mem_expand_pte(struct addr_space* as, vaddr_t va, size_t lvl)
{
    if (as->pt.dscr->lvls - 1 <= lvl) {
        return;  // 没有更多级别可以膨胀
    }

    pte_t* pte = pt_get_pte(&as->pt, lvl, va);

    // 只能膨胀如果存在且不是下一级页表
    if (pte != NULL && !pte_table(&as->pt, pte, lvl)) {
        pte_t pte_val = *pte;  // 保存原始 PTE
        bool rsv = pte_check_rsw(pte, PTE_RSW_RSRV);
        bool vld = pte_valid(pte);
        pte = mem_alloc_pt(as, pte, lvl, va);

        if (vld || rsv) {
            // 如果之前有效且不是页表，必须是大页
            // 填充新膨胀的页表以具有相同的映射
            
            // 使旧大页的 TLB 条目无效
            tlb_inv_va(&cpu()->as, va);

            // 遍历新下一级页表以复制原始映射
            lvl++;
            paddr_t paddr = pte_addr(&pte_val);
            size_t entry = pt_getpteindex(&as->pt, pte, lvl);
            size_t nentries = pt_nentries(&as->pt, lvl);
            size_t lvlsz = pt_lvlsize(&as->pt, lvl);
            pte_type_t type = pt_page_type(&as->pt, lvl);
            pte_flags_t flags = (as->type == AS_HYP ? PTE_HYP_FLAGS : PTE_VM_FLAGS);

            while (entry < nentries) {
                if (vld) {
                    pte_set(pte, paddr, type, flags);
                } else if (rsv) {
                    pte_set_rsw(pte, PTE_RSW_RSRV);
                }
                pte++;
                entry++;
                paddr += lvlsz;
            }

            fence_sync_write();
        }
    }
}
```

**页表膨胀机制：**
- **需求驱动**：当需要更细粒度的映射时自动膨胀
- **状态保持**：保留原始大页的映射和权限
- **TLB 同步**：使旧映射的 TLB 条目无效
- **原子操作**：使用内存屏障确保一致性

#### C.1.4 虚拟页分配

```c
vaddr_t mem_alloc_vpage(struct addr_space* as, enum AS_SEC section, vaddr_t at, size_t n)
{
    size_t lvl = 0;
    size_t entry = 0;
    size_t nentries = 0;
    size_t lvlsze = 0;
    size_t count = 0;
    vaddr_t addr = INVALID_VA;
    vaddr_t vpage = INVALID_VA;
    vaddr_t top = MAX_VA;
    pte_t* pte = NULL;
    bool failed = false;

    struct section* sec = &sections[as->type].sec[section];
    if (at != INVALID_VA) {
        if (sec != mem_find_sec(as, at)) {
            return INVALID_VA;
        }
        addr = at;
    } else {
        addr = sec->beg;
    }
    top = sec->end;

    if (addr > top || !IS_ALIGNED(addr, PAGE_SIZE)) {
        return INVALID_VA;
    }

    spin_lock(&as->lock);
    if (sec->shared) {
        spin_lock(&sec->lock);
    }

    while (count < n && !failed) {
        // 检查地址空间中是否还有足够空间
        size_t full_as = (addr == 0) && (top == MAX_VA);
        if (!full_as && (((top + 1 - addr) / PAGE_SIZE) < n)) {
            vpage = INVALID_VA;
            failed = true;
            break;
        }

        pte = pt_get_pte(&as->pt, lvl, addr);
        entry = pt_getpteindex(&as->pt, pte, lvl);
        nentries = pt_nentries(&as->pt, lvl);
        lvlsze = pt_lvlsize(&as->pt, lvl);

        while ((entry < nentries) && (count < n) && !failed) {
            if (pte_check_rsw(pte, PTE_RSW_RSRV) ||
                (pte_valid(pte) && !pte_table(&as->pt, pte, lvl))) {
                count = 0;
                vpage = INVALID_VA;
                if (at != INVALID_VA) {
                    failed = true;
                    break;
                }
            } else if (!pte_valid(pte)) {
                if (pte_allocable(as, pte, lvl, n - count, addr)) {
                    if (count == 0) {
                        vpage = addr;
                    }
                    count += (lvlsze / PAGE_SIZE);
                } else {
                    if (mem_alloc_pt(as, pte, lvl, addr) == NULL) {
                        ERROR("failed to alloc page table");
                    }
                }
            }

            if (pte_table(&as->pt, pte, lvl)) {
                lvl++;
                break;
            } else {
                pte++;
                addr += lvlsze;
                if (++entry >= nentries) {
                    lvl = 0;
                    break;
                }
            }
        }
    }

    // 标记页表项为保留
    if (vpage != INVALID_VA && !failed) {
        count = 0;
        addr = vpage;
        lvl = 0;
        while (count < n) {
            for (lvl = 0; lvl < as->pt.dscr->lvls; lvl++) {
                pte = pt_get_pte(&as->pt, lvl, addr);
                if (!pte_valid(pte)) {
                    break;
                }
            }
            pte_set_rsw(pte, PTE_RSW_RSRV);
            addr += pt_lvlsize(&as->pt, lvl);
            count += pt_lvlsize(&as->pt, lvl) / PAGE_SIZE;
        }
    }

    if (sec->shared) {
        spin_unlock(&sec->lock);
    }
    spin_unlock(&as->lock);

    return vpage;
}
```

**分配策略：**
- **固定地址**：支持在指定地址分配
- **自动分配**：从段起始地址自动搜索
- **空间检查**：确保有足够的连续虚拟地址空间
- **保留标记**：使用 RSW 位标记已分配但未映射的页

#### C.1.5 内存映射

```c
static bool mem_map(struct addr_space* as, vaddr_t va, struct ppages* ppages, size_t num_pages,
    mem_flags_t flags)
{
    size_t count = 0;
    pte_t* pte = NULL;
    vaddr_t vaddr = va & ~((vaddr_t)(PAGE_SIZE - 1));

    struct section* sec = mem_find_sec(as, vaddr);

    if ((sec == NULL) || (sec != mem_find_sec(as, vaddr + num_pages * PAGE_SIZE - 1))) {
        return false;
    }

    spin_lock(&as->lock);
    if (sec->shared) {
        spin_lock(&sec->lock);
    }

    struct ppages temp_ppages;
    if (ppages == NULL && !all_clrs(as->colors)) {
        temp_ppages = mem_alloc_ppages(as->colors, num_pages, MEM_ALIGN_NOT_REQ);
        if (temp_ppages.num_pages < num_pages) {
            ERROR("failed to alloc colored physical pages");
        }
        ppages = &temp_ppages;
    }

    if (ppages && !all_clrs(ppages->colors)) {
        // 着色映射：膨胀页表到最后一级
        size_t index = 0;
        mem_inflate_pt(as, vaddr, num_pages * PAGE_SIZE);
        for (size_t i = 0; i < ppages->num_pages; i++) {
            pte = pt_get_pte(&as->pt, as->pt.dscr->lvls - 1, vaddr);
            index = pp_next_clr(ppages->base, index, ppages->colors);
            paddr_t paddr = ppages->base + (index * PAGE_SIZE);
            pte_set(pte, paddr, PTE_PAGE, flags);
            vaddr += PAGE_SIZE;
            index++;
        }
    } else {
        // 非着色映射：可以使用大页
        paddr_t paddr = ppages ? ppages->base : 0;
        while (count < num_pages) {
            size_t lvl = 0;
            for (lvl = 0; lvl < as->pt.dscr->lvls; lvl++) {
                pte = pt_get_pte(&as->pt, lvl, vaddr);
                if (pt_lvl_terminal(&as->pt, lvl)) {
                    if (pt_pte_mappable(as, pte, lvl, num_pages - count, vaddr,
                            ppages ? paddr : 0)) {
                        break;
                    } else if (!pte_valid(pte)) {
                        mem_alloc_pt(as, pte, lvl, vaddr);
                    } else if (!pte_table(&as->pt, pte, lvl)) {
                        ERROR("trying to override previous mapping");
                    }
                }
            }

            size_t entry = pt_getpteindex(&as->pt, pte, lvl);
            size_t nentries = pt_nentries(&as->pt, lvl);
            size_t lvlsz = pt_lvlsize(&as->pt, lvl);

            while ((entry < nentries) && (count < num_pages) &&
                (num_pages - count >= lvlsz / PAGE_SIZE)) {
                if (ppages == NULL) {
                    struct ppages temp =
                        mem_alloc_ppages(as->colors, lvlsz / PAGE_SIZE, MEM_ALIGN_REQ);
                    if (temp.num_pages < lvlsz / PAGE_SIZE) {
                        if (lvl == (as->pt.dscr->lvls - 1)) {
                            ERROR("failed to alloc physical pages");
                        } else {
                            pte = pt_get_pte(&as->pt, lvl, vaddr);
                            if (!pte_valid(pte)) {
                                mem_alloc_pt(as, pte, lvl, vaddr);
                            }
                            break;
                        }
                    }
                    paddr = temp.base;
                }
                pte_set(pte, paddr, pt_page_type(&as->pt, lvl), flags);
                vaddr += lvlsz;
                paddr += lvlsz;
                count += lvlsz / PAGE_SIZE;
                pte++;
                entry++;
            }
        }
    }

    fence_sync();

    if (sec->shared) {
        spin_unlock(&sec->lock);
    }
    spin_unlock(&as->lock);

    return true;
}
```

**映射策略：**
- **着色映射**：强制使用 4KB 页以确保颜色匹配
- **大页优化**：非着色映射使用大页减少 TLB 压力
- **自动分配**：未指定物理页时自动分配
- **内存屏障**：确保映射对其他 CPU 可见

#### C.1.6 内存重新着色

```c
bool mem_map_reclr(struct addr_space* as, vaddr_t va, struct ppages* ppages, size_t num_pages,
    mem_flags_t flags)
{
    if (ppages == NULL) {
        ERROR("no indication on what to recolor");
    }

    // 计算需要重新着色的页数
    size_t reclrd_num = num_pages / (COLOR_NUM * COLOR_SIZE) * COLOR_SIZE *
        bit_count(~(as->colors & BIT_MASK(0, COLOR_NUM)));
    size_t clr_offset = (ppages->base / PAGE_SIZE) % (COLOR_NUM * COLOR_SIZE);
    for (size_t i = 0; i < (num_pages % (COLOR_NUM * COLOR_SIZE)); i++) {
        if (!bit_get(as->colors, (i + clr_offset) / COLOR_SIZE % COLOR_NUM)) {
            reclrd_num++;
        }
    }

    // 如果地址空间未分配特定颜色或没有页需要重新着色，委托给普通映射
    if (all_clrs(as->colors) || (reclrd_num == 0)) {
        return mem_map(as, va, ppages, num_pages, flags);
    }

    // 在 hypervisor 地址空间中分配重新着色的页
    vaddr_t reclrd_va_base = mem_alloc_vpage(&cpu()->as, SEC_HYP_VM, INVALID_VA, reclrd_num);
    struct ppages reclrd_ppages = mem_alloc_ppages(as->colors, reclrd_num, MEM_ALIGN_NOT_REQ);
    mem_map(&cpu()->as, reclrd_va_base, &reclrd_ppages, reclrd_num, PTE_HYP_FLAGS);

    // 将原始镜像映射到 hypervisor 地址空间
    vaddr_t phys_va_base = mem_alloc_vpage(&cpu()->as, SEC_HYP_VM, INVALID_VA, num_pages);
    mem_map(&cpu()->as, phys_va_base, ppages, num_pages, PTE_HYP_FLAGS);

    pte_t* pte = NULL;
    vaddr_t vaddr = va & ~((vaddr_t)(PAGE_SIZE - 1));
    paddr_t paddr = ppages->base;
    vaddr_t clrd_vaddr = reclrd_va_base;
    vaddr_t phys_va = phys_va_base;
    size_t index = 0;

    // 膨胀保留的页表到最后一级
    mem_inflate_pt(as, vaddr, num_pages * PAGE_SIZE);

    for (size_t i = 0; i < num_pages; i++) {
        pte = pt_get_pte(&as->pt, as->pt.dscr->lvls - 1, vaddr);

        // 如果镜像页已经是目标颜色，直接映射。否则先复制到分配的页
        if (bit_get(as->colors, ((i + clr_offset) / COLOR_SIZE % COLOR_NUM))) {
            pte_set(pte, paddr, PTE_PAGE, flags);
        } else {
            memcpy((void*)clrd_vaddr, (void*)phys_va, PAGE_SIZE);
            index = pp_next_clr(reclrd_ppages.base, index, as->colors);
            paddr_t clrd_paddr = reclrd_ppages.base + (index * PAGE_SIZE);
            pte_set(pte, clrd_paddr, PTE_PAGE, flags);

            clrd_vaddr += PAGE_SIZE;
            index++;
        }
        paddr += PAGE_SIZE;
        phys_va += PAGE_SIZE;
        vaddr += PAGE_SIZE;
    }

    // 刷新新分配的着色页
    cache_flush_range(reclrd_va_base, reclrd_num * PAGE_SIZE);

    // 释放原始镜像的未着色页
    struct ppages unused_pages = { .base = ppages->base,
        .num_pages = reclrd_num,
        .colors = ~as->colors };
    mem_free_ppages(&unused_pages);

    mem_unmap(&cpu()->as, reclrd_va_base, reclrd_num, MEM_DONT_FREE_PAGES);
    mem_unmap(&cpu()->as, phys_va_base, num_pages, MEM_DONT_FREE_PAGES);

    return true;
}
```

**重新着色流程：**
1. **分析需求**：计算需要重新着色的页数
2. **分配目标页**：分配符合目标颜色的物理页
3. **映射源页**：将原始镜像映射到 hypervisor 空间
4. **复制内容**：将需要重新着色的页复制到目标页
5. **更新映射**：更新 VM 地址空间的映射
6. **清理资源**：释放临时映射和未使用的页

---

## 附录 D：设备驱动和平台支持（第四次迭代）

### D.1 UART 驱动架构

Bao 实现了统一的 UART 驱动接口，支持多种 UART 控制器。

#### D.1.1 PL011 UART 驱动 ([`pl011_uart.c`](hyper/bao-hypervisor/src/platform/drivers/pl011_uart/pl011_uart.c:1))

**驱动初始化：**

```c
void uart_init(volatile struct Pl011_Uart_hw* ptr_uart)
{
    uint32_t lcrh_reg;

    // 首先禁用所有功能
    ptr_uart->control = 0x0;

    // 禁用 FIFO
    lcrh_reg = ptr_uart->line_control;
    lcrh_reg &= ~UART_LCR_FEN;
    ptr_uart->line_control = lcrh_reg;

    // 默认波特率 = 115200
    uint32_t baud_rate = UART_BAUD_RATE;
    uart_set_baud_rate(ptr_uart, baud_rate);

    // 设置 UART 为 8 位，1 个停止位，无奇偶校验，启用 FIFO
    ptr_uart->line_control = (UART_LCR_WLEN_8 | UART_LCR_FEN);

    // 启用 UART，启用 TX 和启用环回
    ptr_uart->control = (UART_CR_UARTEN | UART_CR_TXE | UART_CR_LBE);

    // 设置接收中断 FIFO 级别为 1/2 满
    ptr_uart->isr_fifo_level_sel = UART_IFLS_RXIFLSEL_1_2;

    ptr_uart->data = 0x0;
    while (ptr_uart->flag & UART_FR_BUSY) { }

    // 启用 RX
    ptr_uart->control = (UART_CR_UARTEN | UART_CR_RXE | UART_CR_TXE);

    // 清除中断
    ptr_uart->isr_clear = (UART_ICR_OEIC | UART_ICR_BEIC | UART_ICR_PEIC | UART_ICR_FEIC);

    // 启用接收和接收超时中断
    ptr_uart->isr_mask = (UART_MIS_RXMIS | UART_MIS_RTMIS);
}
```

**波特率设置：**

```c
void uart_set_baud_rate(volatile struct Pl011_Uart_hw* ptr_uart, uint32_t baud_rate)
{
    uint32_t temp;
    uint32_t ibrd;
    uint32_t mod;
    uint32_t fbrd;

    if (baud_rate == 0) {
        baud_rate = UART_BAUD_RATE;
    }

    // 设置波特率，IBRD = UART_CLK / (16 * BAUD_RATE)
    // FBRD = ROUND((64 * MOD(UART_CLK,(16 * BAUD_RATE))) / (16 * BAUD_RATE))
    temp = 16 * baud_rate;
    ibrd = UART_CLK / temp;
    mod = UART_CLK % temp;
    fbrd = (4 * mod) / baud_rate;

    // 设置波特率除数器的值
    ptr_uart->integer_br = ibrd;
    ptr_uart->fractional_br = fbrd;
}
```

**UART 操作：**

```c
// 接收字符
uint32_t uart_getc(volatile struct Pl011_Uart_hw* ptr_uart)
{
    uint32_t data = 0;

    // 等待直到 FIFO 中有数据
    while (!(ptr_uart->flag & UART_FR_RXFE)) { }

    data = ptr_uart->data;
    return data;
}

// 发送字符
void uart_putc(volatile struct Pl011_Uart_hw* ptr_uart, int8_t c)
{
    // 等待直到 txFIFO 未满
    while (ptr_uart->flag & UART_FR_TXFF) { }

    ptr_uart->data = (uint32_t)c;
}
```

**UART 寄存器映射：**

| 寄存器 | 偏移 | 功能 |
|---------|------|------|
| `data` | 0x00 | 数据寄存器 |
| `status_error` | 0x04 | 接收状态/错误清除寄存器 |
| `flag` | 0x18 | 标志寄存器 |
| `integer_br` | 0x24 | 整数波特率寄存器 |
| `fractional_br` | 0x28 | 分数波特率寄存器 |
| `line_control` | 0x2C | 行控制寄存器 |
| `control` | 0x30 | 控制寄存器 |
| `isr_fifo_level_sel` | 0x34 | 中断 FIFO 级别选择寄存器 |
| `isr_mask` | 0x38 | 中断掩码设置/清除寄存器 |
| `raw_isr_status` | 0x3C | 原始中断状态寄存器 |
| `masked_isr_status` | 0x40 | 掩码中断状态寄存器 |
| `isr_clear` | 0x44 | 中断清除寄存器 |
| `DMA_control` | 0x48 | DMA 控制寄存器 |

#### D.1.2 其他 UART 驱动

Bao 支持多种 UART 驱动：

| 驱动 | 平台 | 特性 |
|------|------|------|
| `8250_uart` | NXP i.MX8 | 8250 UART 控制器 |
| `cmsdk_uart` | ARM CMSDK | CMSDK UART 接口 |
| `imx_uart` | NXP i.MX | i.MX UART 控制器 |
| `nxp_uart` | NXP | NXP UART 控制器 |
| `sbi_uart` | RISC-V | SBI UART 接口 |
| `zynq_uart` | Xilinx Zynq | Zynq UART 控制器 |

### D.2 平台描述结构

#### D.2.1 平台结构定义

所有平台都使用统一的结构来描述硬件资源：

```c
struct platform {
    size_t cpu_num;                    // CPU 数量
    size_t region_num;                 // 内存区域数量
    struct mem_region* regions;          // 内存区域数组
    struct {
        paddr_t base;                  // 控制台基地址
    } console;
    struct {
        struct {
            paddr_t gicd_addr;          // GIC 分发器地址
            paddr_t gicc_addr;          // GIC CPU 接口地址
            paddr_t gich_addr;          // GIC 虚拟化接口地址
            paddr_t gicv_addr;          // GIC 虚拟 CPU 接口地址
            paddr_t gicr_addr;          // GIC 重分发器地址
            irqid_t maintenance_id;     // GIC 维护中断 ID
        } gic;
        // RISC-V 特定配置
        struct {
            paddr_t plic_base;           // PLIC 基地址
            paddr_t aia.aplic.base;     // APLIC 基地址
            paddr_t aia.imsic.base;     // IMSIC 基地址
            size_t aia.imsic.num_msis; // IMSIC 数量
            paddr_t aclint_sswi.base;   // ACLINT SSWI 基地址
        } irqc;
    } arch;
};
```

#### D.2.2 ARMv8-A 平台示例

**FVP-A 平台 ([`fvp-a/fvpa_desc.c`](hyper/bao-hypervisor/src/platform/fvp-a/fvpa_desc.c:1))：**

```c
struct platform platform = {
    .cpu_num = 4,
    .region_num = 1,
    .regions = (struct mem_region[]){
        {
            // DRAM, 0GB-2GB
            .base = 0x80000000,
            .size = 0x80000000,
        },
    },

    .console = {
        .base = 0x1C090000,  // UART0 (PL011)
    },

    .arch = {
        .gic = {
            .gicd_addr = 0x2F000000,
            .gicc_addr = 0x2C000000,
            .gich_addr = 0x2C010000,
            .gicv_addr = 0x2C02F000,
            .gicr_addr = 0x2F100000,
            .maintenance_id = 25,
        },
    },
};
```

**HiKey 960 平台 ([`hikey960/hikey960_desc.c`](hyper/bao-hypervisor/src/platform/hikey960/hikey960_desc.c:1))：**
- 8 核 HiSilicon Kirin 960
- GICv3 中断控制器
- UART 控制台

**i.MX8QM 平台 ([`imx8qm/imx8qm_desc.c`](hyper/bao-hypervisor/src/platform/imx8qm/imx8qm_desc.c:1))：**
- NXP i.MX8QM 处理器
- GICv3 中断控制器
- SMMUv2 IOMMU

**QEMU AArch64 Virt 平台 ([`qemu-aarch64-virt/virt_desc.c`](hyper/bao-hypervisor/src/platform/qemu-aarch64-virt/virt_desc.c:1))：**

```c
struct platform platform = {
    .cpu_num = 4,
    .region_num = 1,
    .regions = (struct mem_region[]){
        {
            .base = 0x40000000,
            .size = 0x100000000,
        },
    },

    .console = {
        .base = 0x9000000
    },

    .arch = {
        .gic = {
            .gicd_addr = 0x08000000,
            .gicc_addr = 0x08010000,
            .gich_addr = 0x08030000,
            .gicv_addr = 0x08040000,
            .gicr_addr = 0x080A0000,
            .maintenance_id = 25,
        },
    },
};
```

#### D.2.3 ARMv8-R 平台示例

**FVP-R 平台 ([`fvp-r/fvpr_desc.c`](hyper/bao-hypervisor/src/platform/fvp-r/fvpr_desc.c:1))：**
- ARM Cortex-R52 处理器
- GICv3 中断控制器
- MPU 内存保护

**MPS3-AN536 平台 ([`mps3-an536/mps3_desc.c`](hyper/bao-hypervisor/src/platform/mps3-an536/mps3_desc.c:1))：**
- ARM Cortex-R52 处理器
- GICv3 中断控制器
- MPS3-AN536 FPGA 平台

#### D.2.4 RISC-V 平台示例

**QEMU RISC-V64 Virt 平台 ([`qemu-riscv64-virt/virt_desc.c`](hyper/bao-hypervisor/src/platform/qemu-riscv64-virt/virt_desc.c:1))：**

```c
struct platform platform = {
    .cpu_num = 4,

    .region_num = 1,
    .regions = (struct mem_region[]){
        {
            .base = 0x80200000,
            .size = QEMU_VIRT_MEM_REG_SIZE,
        },
    },

    .arch = {
#if (IRQC == PLIC)
        .irqc.plic.base = 0xc000000,
#elif (IRQC == APLIC)
        .irqc.aia.aplic.base = 0xd000000,
#elif (IRQC == AIA)
        .irqc.aia.aplic.base = 0xd000000,
        .irqc.aia.imsic.base = 0x28000000,
        .irqc.aia.imsic.num_msis = 255,
#else
#error "unknown IRQC type " IRQC
#endif
    },
};
```

**RISC-V 中断控制器支持：**

| 控制器 | 基地址 | 特性 |
|---------|--------|------|
| PLIC | 0xc000000 | Platform-Level Interrupt Controller |
| APLIC | 0xd000000 | Advanced Platform-Level Interrupt Controller |
| IMSIC | 0x28000000 | Incoming MSI Controller |
| ACLINT SSWI | 0x2f00000 | ACLINT SSWI |

**QEMU RISC-V32 Virt 平台 ([`qemu-riscv32-virt/virt_desc.c`](hyper/bao-hypervisor/src/platform/qemu-riscv32-virt/virt_desc.c:1))：**
- 32 位 RISC-V 处理器
- PLIC 中断控制器
- 1GB 内存限制（由于 hypervisor 保留限制）

### D.3 平台抽象层

#### D.3.1 平台头文件

每个平台都有一个 `plat/platform.h` 头文件，定义平台特定的配置：

```c
#ifndef __PLAT_PLATFORM_H__
#define __PLAT_PLATFORM_H__

#include <drivers/pl011_uart.h>  // 或其他 UART 驱动

#endif
```

**RISC-V 平台特定定义：**

```c
#ifndef __PLAT_PLATFORM_H__
#define __PLAT_PLATFORM_H__

#include <drivers/sbi_uart.h>

#define CPU_EXT_SSTC 1
#define IPIC_SBI     (1)
#define IPIC_ACLINT  (2)

#endif
```

#### D.3.2 PSCI 支持

ARMv8 平台提供 PSCI (Power State Coordination Interface) 支持：

```c
// PSCI 函数 ID
#define PSCI_VERSION           0x84000000
#define PSCI_CPU_ON            0x84000003
#define PSCI_CPU_OFF           0x84000002
#define PSCI_CPU_SUSPEND       0x84000001
#define PSCI_AFFINITY_INFO     0x84000004
```

### D.4 驱动接口统一

#### D.4.1 UART 驱动接口

所有 UART 驱动实现统一的接口：

```c
// 禁用 UART
void uart_disable(volatile struct Pl011_Uart_hw* ptr_uart);

// 启用 UART
void uart_enable(volatile struct Pl011_Uart_hw* ptr_uart);

// 设置波特率
void uart_set_baud_rate(volatile struct Pl011_Uart_hw* ptr_uart, uint32_t baud_rate);

// 初始化 UART
void uart_init(volatile struct Pl011_Uart_hw* ptr_uart);

// 接收字符
uint32_t uart_getc(volatile struct Pl011_Uart_hw* ptr_uart);

// 发送字符
void uart_putc(volatile struct Pl011_Uart_hw* ptr_uart, int8_t c);
```

#### D.4.2 平台接口

平台抽象层提供统一的接口：

```c
// 平台初始化
void platform_init(void);

// 平台特定初始化
void platform_arch_init(void);

// PSCI 初始化
void psci_init(void);

// IOMMU 初始化
void iommu_init(void);
```

### D.5 支持的平台矩阵

| 平台 | 架构 | CPU | GIC | 中断控制器 | UART | IOMMU |
|------|-------|-----|-----|-----------|------|-------|
| FVP-A | ARMv8-A | 4 | GICv3 | PL011 | - |
| FVP-R | ARMv8-R | 4 | GICv3 | PL011 | - |
| HiKey 960 | ARMv8-A | 8 | GICv3 | - | - |
| i.MX8QM | ARMv8-A | 4 | GICv3 | - | SMMUv2 |
| MPS3-AN536 | ARMv8-R | 4 | GICv3 | - | - |
| QEMU AArch64 | ARMv8-A | 4 | GICv3 | - | - |
| QEMU RISC-V64 | RISC-V | 4 | - | SBI | - |
| QEMU RISC-V32 | RISC-V | 4 | PLIC | SBI | - |
| RPi 4 | ARMv8-A | 4 | GICv3 | PL011 | - |
| TX2 | ARMv8-A | 4 | GICv3 | - | - |
| Ultra96 | ARMv8-A | 4 | GICv3 | - | - |
| ZCU102 | ARMv8-A | 4 | GICv3 | - | - |
| ZCU104 | ARMv8-A | 4 | GICv3 | - | - |

### D.6 平台支持总结

#### D.6.1 设计优势

1. **模块化设计**：
   - 每个平台独立目录
   - 统一的驱动接口
   - 平台特定配置隔离

2. **可扩展性**：
   - 易于添加新平台
   - 驱动可复用
   - 配置驱动架构

3. **硬件抽象**：
   - 统一的平台结构
   - 架构特定配置
   - 设备树支持

4. **多架构支持**：
   - ARMv8-A/R 支持
   - RISC-V RV32/RV64 支持
   - 统一的虚拟化接口

#### D.6.2 优化建议

1. **驱动优化**：
   - 实现驱动热插拔支持
   - 优化 UART FIFO 管理
   - 添加 DMA 支持

2. **平台抽象优化**：
   - 统一设备树解析
   - 实现动态设备发现
   - 改进 PSCI 实现

3. **中断优化**：
   - 优化中断路由
   - 实现中断亲和性
   - 改进中断延迟

4. **IOMMU 优化**：
   - 扩展 IOMMU 支持
   - 优化地址转换
   - 实现设备隔离

#### C.1.7 Hypervisor 着色

```c
void mem_color_hypervisor(const paddr_t load_addr, struct mem_region* root_region)
{
    static volatile pte_t shared_pte;
    vaddr_t va = INVALID_VA;
    struct cpu* cpu_new;
    struct ppages p_cpu;
    struct ppages p_image;
    struct ppages p_bitmap;

    size_t image_load_size = (size_t)(&_image_load_end - &_image_start);
    size_t image_noload_size = (size_t)(&_image_end - &_image_load_end);
    size_t image_size = image_load_size + image_noload_size;
    size_t vm_image_size = (size_t)(&_vm_image_end - &_vm_image_start);
    size_t cpu_boot_size = mem_cpu_boot_alloc_size();
    struct page_pool* root_pool = &root_region->page_pool;
    size_t bitmap_size =
        (root_pool->num_pages / (8 * PAGE_SIZE) + !!(root_pool->num_pages % (8 * PAGE_SIZE) != 0)) *
        PAGE_SIZE;
    colormap_t colors = config.hyp.colors;

    // 设置 hypervisor 颜色
    cpu()->as.colors = config.hyp.colors;

    // 将 CPU 空间复制到着色区域
    cpu_new = copy_space((void*)BAO_CPU_BASE, sizeof(struct cpu), &p_cpu);
    as_init(&cpu_new->as, AS_HYP_CPY, NULL, colors);
    va = mem_alloc_vpage(&cpu_new->as, SEC_HYP_PRIVATE, (vaddr_t)BAO_CPU_BASE,
        NUM_PAGES(sizeof(struct cpu)));
    if (va != (vaddr_t)BAO_CPU_BASE) {
        ERROR("Can't allocate virtual address for cpuspace");
    }
    mem_map(&cpu_new->as, va, &p_cpu, NUM_PAGES(sizeof(struct cpu)), PTE_HYP_FLAGS);

    // 映射根页表到新地址空间
    paddr_t p_root_pt_addr;
    vaddr_t v_root_pt_addr;
    size_t root_pt_num_pages = NUM_PAGES(pt_size(&cpu_new->as.pt, 0));
    mem_translate(&cpu()->as, (vaddr_t)cpu_new->as.pt.root, &p_root_pt_addr);
    v_root_pt_addr = mem_alloc_vpage(&cpu_new->as, SEC_HYP_PRIVATE, INVALID_VA, root_pt_num_pages);
    if (va == INVALID_VA) {
        ERROR("Can't allocate virtuall address space for root page table");
    }
    struct ppages p_root_pt_pages = mem_ppages_get(p_root_pt_addr, root_pt_num_pages);
    mem_map(&cpu_new->as, v_root_pt_addr, &p_root_pt_pages, root_pt_num_pages, PTE_HYP_FLAGS);

    // 复制 Hypervisor 镜像到着色区域
    if (cpu_is_master()) {
        copy_space(&_image_start, image_size, &p_image);
        va = mem_alloc_vpage(&cpu_new->as, SEC_HYP_IMAGE, (vaddr_t)&_image_start,
            NUM_PAGES(image_size));

        if (va != (vaddr_t)&_image_start) {
            ERROR("Can't allocate virtual address for Bao Image");
        }

        mem_map(&cpu_new->as, va, &p_image, NUM_PAGES(image_size), PTE_HYP_FLAGS);
        shared_pte = pte_addr(pt_get_pte(&cpu_new->as.pt, 0, (vaddr_t)&_image_start));
    } else {
        pte_t* image_pte = pt_get_pte(&cpu_new->as.pt, 0, (vaddr_t)&_image_start);
        while (shared_pte == 0) { }
        pte_set(image_pte, (paddr_t)shared_pte, PTE_TABLE, PTE_HYP_FLAGS);
    }

    cpu_sync_barrier(&cpu_glb_sync);

    // 主核复制根页池位图
    if (cpu_is_master()) {
        copy_space((void*)root_pool->bitmap, bitmap_size, &p_bitmap);
        va = mem_alloc_vpage(&cpu_new->as, SEC_HYP_GLOBAL, (vaddr_t)root_pool->bitmap,
            NUM_PAGES(bitmap_size));

        if (va != (vaddr_t)root_pool->bitmap) {
            ERROR("Can't allocate address for cpu interface");
        }

        mem_map(&cpu_new->as, va, &p_bitmap, NUM_PAGES(bitmap_size), PTE_HYP_FLAGS);
    }
    cpu_sync_barrier(&cpu_glb_sync);

    // 切换到新地址空间
    switch_space(cpu_new, p_root_pt_addr);

    // 刷新新物理页到主内存
    cache_flush_range((vaddr_t)&_image_start, image_size);
    cache_flush_range((vaddr_t)&_cpu_private_beg, sizeof(struct cpu));

    // 重新初始化同步对象
    if (cpu_is_master()) {
        cpu_sync_init(&cpu_glb_sync, platform.cpu_num);
        shared_pte = 0;
    } else {
        while (shared_pte != 0) { }
    }

    as_init(&cpu()->as, AS_HYP, (void*)v_root_pt_addr, colors);

    // 清除已复制的旧区域
    if (cpu_is_master()) {
        p_image = mem_ppages_get(load_addr, NUM_PAGES(image_load_size));
        va = mem_alloc_vpage(&cpu()->as, SEC_HYP_GLOBAL, INVALID_VA, p_image.num_pages);
        mem_map(&cpu()->as, va, &p_image, p_image.num_pages, PTE_HYP_FLAGS);
        memset((void*)va, 0, p_image.num_pages * PAGE_SIZE);
        mem_unmap(&cpu()->as, va, p_image.num_pages, MEM_FREE_PAGES);

        p_image = mem_ppages_get(load_addr + image_load_size + vm_image_size,
            NUM_PAGES(image_noload_size));
        va = mem_alloc_vpage(&cpu()->as, SEC_HYP_GLOBAL, INVALID_VA, p_image.num_pages);
        mem_map(&cpu()->as, va, &p_image.num_pages, PTE_HYP_FLAGS);
        memset((void*)va, 0, p_image.num_pages * PAGE_SIZE);
        mem_unmap(&cpu()->as, va, p_image.num_pages, MEM_FREE_PAGES);

        p_bitmap = mem_ppages_get(load_addr + image_size + vm_image_size +
                (cpu_boot_size * platform.cpu_num),
            NUM_PAGES(bitmap_size));

        va = mem_alloc_vpage(&cpu()->as, SEC_HYP_GLOBAL, INVALID_VA, p_bitmap.num_pages);
        mem_map(&cpu()->as, va, &p_bitmap, p_bitmap.num_pages, PTE_HYP_FLAGS);
        memset((void*)va, 0, p_bitmap.num_pages * PAGE_SIZE);
        mem_unmap(&cpu()->as, va, p_bitmap.num_pages, MEM_FREE_PAGES);
    }

    p_cpu = mem_ppages_get(load_addr + image_size + vm_image_size + (cpu_boot_size * cpu()->id),
        cpu_boot_size / PAGE_SIZE);
    va = mem_alloc_vpage(&cpu()->as, SEC_HYP_PRIVATE, INVALID_VA, p_cpu.num_pages);
    mem_map(&cpu()->as, va, &p_cpu, p_bitmap.num_pages, PTE_HYP_FLAGS);
    memset((void*)va, 0, p_cpu.num_pages * PAGE_SIZE);
    mem_unmap(&cpu()->as, va, p_cpu.num_pages, MEM_DONT_FREE_PAGES);
}
```

**Hypervisor 着色流程：**
1. **创建新地址空间**：为着色的 hypervisor 创建新地址空间
2. **复制 CPU 空间**：复制 CPU 结构到着色区域
3. **复制镜像**：主核复制 hypervisor 镜像到着色区域
4. **同步所有 CPU**：确保所有 CPU 都完成复制
5. **切换地址空间**：切换到新的着色地址空间
6. **清理旧区域**：释放旧的未着色内存

### C.2 MPU 内存保护 ([`mpu/mem.c`](hyper/bao-hypervisor/src/core/mpu/mem.c:1))

#### C.2.1 VMPU (Virtual MPU) 管理

**VMPU 条目结构：**

```c
struct mpe {
    struct mp_region region;  // MPU 区域
    mpid_t mpid;            // MPU ID
    enum mpe_state state;   // 条目状态
    bool lock;               // 是否锁定
};
```

**VMPU 状态：**

```c
enum mpe_state {
    MPE_S_FREE,    // 空闲
    MPE_S_INVALID,  // 无效
    MPE_S_VALID    // 有效
};
```

#### C.2.2 VMPU 条目分配

```c
static mpid_t mem_vmpu_allocate_entry(struct addr_space* as)
{
    mpid_t mpid = INVALID_MPID;

    for (mpid_t i = 0; i < VMPU_NUM_ENTRIES; i++) {
        struct mpe* mpe = mem_vmpu_get_entry(as, i);
        if (mpe->state == MPE_S_FREE) {
            mpid = i;
            mpe->state = MPE_S_INVALID;
            break;
        }
    }

    return mpid;
}
```

#### C.2.3 VMPU 条目设置

```c
static void mem_vmpu_set_entry(struct addr_space* as, mpid_t mpid, struct mp_region* mpr,
    bool locked)
{
    struct mpe* mpe = mem_vmpu_get_entry(as, mpid);

    mpe->region.base = mpr->base;
    mpe->region.size = mpr->size;
    mpe->region.mem_flags = mpr->mem_flags;
    mpe->region.as_sec = mpr->as_sec;
    mpe->state = MPE_S_VALID;
    mpe->mpid = mpid;
    mpe->lock = locked;

    // 按地址顺序插入到有序列表
    list_insert_ordered(&as->vmpu.ordered_list, (node_t*)&as->vmpu.node[mpid], vmpu_node_cmp);
}
```

#### C.2.4 VMPU 区域映射

```c
bool mem_map(struct addr_space* as, struct mp_region* mpr, bool broadcast, bool locked)
{
    bool mapped = false;
    mpid_t mpid = INVALID_MPID;

    if (mpr->size == 0) {
        return true;
    }

    if ((mpr->size % mpu_granularity()) != 0) {
        ERROR("trying to set mpu region which is not a multiple of granularity");
    }

    spin_lock(&as->lock);

    // 检查重叠区域
    if (mem_vmpu_find_overlapping_region(as, mpr) == INVALID_MPID) {
        mpid = mem_vmpu_allocate_entry(as);
        if (mpid != INVALID_MPID) {
            mapped = mem_vmpu_insert_region(as, mpid, mpr, broadcast, locked);
        } else {
            mem_vmpu_deallocate_entry(as, mpid);
        }
    }

    // 如果未锁定，尝试合并连续区域
    if (mapped && !locked) {
        mem_vmpu_coalesce_contiguous(as, broadcast);
    }

    spin_unlock(&as->lock);

    return mapped;
}
```

**MPU 映射特性：**
- **粒度对齐**：区域大小必须是 MPU 粒度的倍数
- **重叠检查**：确保新区域不与现有区域重叠
- **自动合并**：合并具有相同权限的连续区域
- **广播机制**：支持跨 CPU 的区域广播

#### C.2.5 VMPU 区域合并

```c
static void mem_vmpu_coalesce_contiguous(struct addr_space* as, bool broadcast)
{
    while (true) {
        bool merge = false;
        mpid_t cur_mpid = INVALID_MPID;
        mpid_t prev_mpid = INVALID_MPID;
        struct mpe* prev_reg;
        struct mpe* cur_reg;

        // 查找可合并的相邻区域
        list_foreach_tail(as->vmpu.ordered_list, struct mpe, cur, prev) {
            if (prev == NULL) {
                continue;
            }
            cur_reg = mem_vmpu_get_entry(as, cur->mpid);
            prev_reg = mem_vmpu_get_entry(as, prev->mpid);

            bool contiguous = prev_reg->region.base + prev_reg->region.size == cur_reg->region.base;
            bool perms_compatible =
                mpu_perms_compatible(prev_reg->region.mem_flags.raw, cur_reg->region.mem_flags.raw);
            bool lock_compatible = !prev_reg->lock && !cur_reg->lock;
            if (contiguous && perms_compatible && lock_compatible) {
                cur_mpid = cur->mpid;
                prev_mpid = prev->mpid;
                merge = true;
                break;
            }
        }

        if (merge) {
            struct mp_region merged_reg = {
                .base = prev_reg->region.base,
                .size = prev_reg->region.size + cur_reg->region.size,
                .mem_flags = cur_reg->region.mem_flags,
            };
            if (mem_vmpu_update_region(as, prev_mpid, merged_reg, broadcast, prev_reg->lock)) {
                mem_vmpu_remove_region(as, cur_mpid, broadcast);
            }
        } else {
            break;
        }
    }
}
```

**合并条件：**
- **地址连续**：前一个区域的结束地址等于后一个区域的起始地址
- **权限兼容**：两个区域具有相同的内存权限
- **锁定兼容**：两个区域都未锁定

#### C.2.6 VMPU 区域广播

```c
static void mem_region_broadcast(struct addr_space* as, struct mp_region* mpr, uint32_t op,
    bool locked)
{
    cpumap_t shared_cpus = mem_section_shared_cpus(as, mpr->as_sec);

    if (shared_cpus == 0) {
        return;
    }

    struct shared_region shared_region = {
        .as_type = as->type,
        .asid = as->id,
        .region = *mpr,
        .lock = locked,
    };

    // 向所有共享 CPU 发送广播消息
    for (cpuid_t cpuid = 0; cpuid < PLAT_CPU_NUM; cpuid++) {
        if ((cpu()->id != cpuid) && bit_get(shared_cpus, cpuid)) {
            struct shared_region* node = objpool_alloc(&shared_region_pool);
            if (node == NULL) {
                ERROR("Failed allocating shared region node");
            }
            *node = shared_region;
            struct cpu_msg msg = { (uint32_t)MEM_PROT_SYNC, op, (uintptr_t)node };
            cpu_send_msg(cpuid, &msg);
        }
    }
}
```

**广播机制：**
- **共享 CPU 检测**：确定哪些 CPU 共享该区域
- **消息传递**：通过 CPU 消息系统发送区域更新
- **操作类型**：支持插入、删除、更新操作
- **对象池**：使用对象池管理共享区域节点

### C.3 缓存着色 ([`cache.c`](hyper/bao-hypervisor/src/core/cache.c:1))

#### C.3.1 缓存着色计算

```c
static void cache_calc_colors(struct cache* dscrp, size_t page_size)
{
    if (dscrp->lvls == 0) {
        return;  // 无缓存
    }

    size_t llc = dscrp->min_shared_lvl;

    // 只支持 PIPT (Physically Indexed, Physically Tagged) 缓存
    if ((dscrp->type[llc] != UNIFIED) || (dscrp->indexed[llc][0] != PIPT)) {
        return;
    }

    size_t llc_way_size = dscrp->numset[llc][UNIFIED] * dscrp->line_size[llc][UNIFIED];

    size_t flc_way_size = 0;
    if (dscrp->type[0] != UNIFIED) {
        flc_way_size = dscrp->numset[0][0] * dscrp->line_size[0][0];
        size_t flc_i_way_size = dscrp->numset[0][1] * dscrp->line_size[0][1];
        if (((dscrp->indexed[0][0] == PIPT) || (flc_i_way_size < flc_way_size)) &&
            (dscrp->indexed[0][1] == PIPT)) {
            flc_way_size = flc_i_way_size;
        }
    }

    size_t llc_num_colors = llc_way_size / page_size;
    size_t flc_num_colors = flc_way_size / page_size;

    COLOR_SIZE = flc_num_colors;
    COLOR_NUM = llc_num_colors / COLOR_SIZE;
}
```

**着色参数：**
- `COLOR_SIZE`: 第一级缓存的颜色数
- `COLOR_NUM`: 最后一级缓存的颜色数
- `llc_way_size`: LLC 路大小
- `flc_way_size`: FLC 路大小

#### C.3.2 缓存枚举

```c
void cache_enumerate(void)
{
    cache_arch_enumerate(&cache_dscr);
    cache_calc_colors(&cache_dscr, PAGE_SIZE);
}
```

**枚举流程：**
1. **架构特定枚举**：调用架构特定的缓存枚举函数
2. **颜色计算**：基于缓存参数计算着色配置
3. **全局变量设置**：更新全局颜色参数

### C.4 IOMMU 管理 ([`mmu/io.c`](hyper/bao-hypervisor/src/core/mmu/io.c:1))

#### C.4.1 IOMMU 初始化

```c
void io_init(void)
{
    iommu_arch_init();
}
```

#### C.4.2 IOMMU VM 初始化

```c
bool io_vm_init(struct vm* vm, const struct vm_config* config)
{
    return iommu_arch_vm_init(vm, config);
}
```

#### C.4.3 IOMMU 设备添加

```c
bool io_vm_add_device(struct vm* vm, deviceid_t dev_id)
{
    bool res = false;

    // 如果 dev_id == 0，假设全局掩码包含此 VM 的相关设备
    if (dev_id != 0) {
        // 流 ID 有效。将此设备与特定 VM 匹配
        res = iommu_arch_vm_add_device(vm, dev_id);
    }

    return res;
}
```

### C.5 内存保护机制总结

#### C.5.1 MMU 模式特性

**关键特性：**
1. **页表管理**：
   - 多级页表遍历
   - 动态页表分配和膨胀
   - 大页支持

2. **地址空间隔离**：
   - 段级访问控制
   - 共享和私有段
   - 锁机制保护

3. **缓存着色**：
   - PIPT 缓存支持
   - 颜色分配算法
   - Hypervisor 着色

4. **虚拟化支持**：
   - Stage-2 地址转换
   - VM 地址空间管理
   - IOMMU 集成

#### C.5.2 MPU 模式特性

**关键特性：**
1. **区域管理**：
   - 固定数量的 MPU 条目
   - 区域合并优化
   - 重叠检查

2. **广播机制**：
   - 跨 CPU 区域同步
   - 消息传递系统
   - 共享区域管理

3. **粒度对齐**：
   - 强制 MPU 粒度对齐
   - 区域大小验证

4. **实时性保证**：
   - 确定性的访问时间
   - 无页表遍历开销
   - 直接硬件保护

#### C.5.3 设计对比

| 特性 | MMU 模式 | MPU 模式 |
|------|-----------|----------|
| 内存管理 | 页表 | 区域 |
| 粒度 | 4KB | 可变（通常 32B-256MB）|
| 灵活性 | 高 | 低 |
| 开销 | 页表遍历 | 直接访问 |
| 适用场景 | 通用系统 | 实时系统 |
| 虚拟化 | Stage-2 转换 | 无硬件支持 |
| 缓存着色 | 支持 | 不适用 |

#### C.5.4 优化建议

1. **MMU 优化**：
   - 使用大页减少 TLB 压力
   - 实现页表共享减少内存占用
   - 优化着色分配算法

2. **MPU 优化**：
   - 实现区域碎片整理
   - 优化广播消息聚合
   - 改进区域合并策略

3. **缓存优化**：
   - 支持更多缓存类型
   - 实现动态颜色分配
   - 优化缓存刷新策略

4. **虚拟化优化**：
   - 优化 Stage-2 页表结构
   - 实现嵌套虚拟化支持
   - 改进 IOMMU 性能

#### B.3.4 SBI 初始化

```c
void sbi_init()
{
    struct sbiret ret;
    
    // 检查 SBI 规范版本
    ret = sbi_get_spec_version();
    if (ret.error != SBI_SUCCESS || ret.value < 2) {
        ERROR("not supported SBI spec version: 0x%x", ret.value);
    }
    
    // 探测所有扩展
    for (size_t i = 0; i < NUM_EXT; i++) {
        ret = sbi_probe_extension(ext_table[i]);
        if (ret.error != SBI_SUCCESS || ret.value == 0) {
            ERROR("sbi does not support ext 0x%x", ext_table[i]);
        }
    }
    
    // 预留定时器中断
    irqc_timer_int_id = interrupts_reserve(TIMR_INT_ID, (irq_handler_t)sbi_timer_irq_handler);
    if (irqc_timer_int_id == INVALID_IRQID) {
        ERROR("Failed to reserve SBI TIMR_INT_ID interrupt");
    }
}
```

#### B.3.5 SBI 消息处理

```c
enum SBI_MSG_EVENTS { SEND_IPI, HART_START };

void sbi_msg_handler(uint32_t event, uint64_t data)
{
    switch (event) {
        case SEND_IPI:
            csrs_hvip_set(HIP_VSSIP);
            break;
        case HART_START: {
            spin_lock(&cpu()->vcpu->arch.sbi_ctx.lock);
            if (cpu()->vcpu->arch.sbi_ctx.state == START_PENDING) {
                vcpu_arch_reset(cpu()->vcpu, cpu()->vcpu->arch.sbi_ctx.start_addr);
                vcpu_writereg(cpu()->vcpu, REG_A1, cpu()->vcpu->arch.sbi_ctx.priv);
                cpu()->vcpu->arch.sbi_ctx.state = STARTED;
            }
            spin_unlock(&cpu()->vcpu->arch.sbi_ctx.lock);
        } break;
        default:
            WARNING("unknown sbi msg");
            break;
    }
}
```

#### B.3.6 SBI VS 处理器

```c
size_t sbi_vs_handler()
{
    unsigned long extid = vcpu_readreg(cpu()->vcpu, REG_A7);
    unsigned long fid = vcpu_readreg(cpu()->vcpu, REG_A6);
    struct sbiret ret;
    
    switch (extid) {
        case SBI_EXTID_BASE:
            ret = sbi_base_handler(fid);
            break;
        case SBI_EXTID_TIME:
            ret = sbi_time_handler(fid);
            break;
        case SBI_EXTID_IPI:
            ret = sbi_ipi_handler(fid);
            break;
        case SBI_EXTID_RFNC:
            ret = sbi_rfence_handler(fid);
            break;
        case SBI_EXTID_HSM:
            ret = sbi_hsm_handler(fid);
            break;
        case SBI_EXTID_BAO:
            ret = sbi_bao_handler(fid);
            break;
        default:
            WARNING("guest issued unsupport sbi extension call (%d)", extid);
            ret.error = SBI_ERR_NOT_SUPPORTED;
            break;
    }
    
    vcpu_writereg(cpu()->vcpu, REG_A0, (unsigned long)ret.error);
    vcpu_writereg(cpu()->vcpu, REG_A1, (unsigned long)ret.value);
    
    return 4;  // 返回指令长度
}
```

### B.4 RISC-V 页表管理 ([`page_table.c`](hyper/bao-hypervisor/src/arch/riscv/page_table.c:1))

#### B.4.1 页表描述符

**RV32 页表描述符：**

```c
struct page_table_dscr sv32_pt_dscr = {
    .lvls = 2,                              // 2 级页表
    .lvl_wdt = (size_t[]){ 32, 22 },
    .lvl_off = (size_t[]){ 22, 12 },
    .lvl_term = (bool[]){ true, true },
};
```

**RV64 页表描述符：**

```c
struct page_table_dscr sv39_pt_dscr = {
    .lvls = 3,                              // 3 级页表
    .lvl_wdt = (size_t[]){ 39, 30, 21 },
    .lvl_off = (size_t[]){ 30, 21, 12 },
    .lvl_term = (bool[]){ true, true, true },
};

struct page_table_dscr sv48_pt_dscr = {
    .lvls = 4,                              // 4 级页表
    .lvl_wdt = (size_t[]){ 48, 39, 30, 21 },
    .lvl_off = (size_t[]){ 39, 30, 21, 12 },
    .lvl_term = (bool[]){ false, true, true, true },
};
```

**Sv39x4 页表描述符：**

```c
struct page_table_dscr sv32x4_pt_dscr = {
    .lvls = 3,                              // 3 级页表
    .lvl_wdt = (size_t[]){ 34, 22 },
    .lvl_off = (size_t[]){ 22, 12 },
    .lvl_term = (bool[]){ true, true },
};

struct page_table_dscr sv39x4_pt_dscr = {
    .lvls = 3,                              // 3 级页表
    .lvl_wdt = (size_t[]){ 39, 30, 21 },
    .lvl_off = (size_t[]){ 30, 21, 12 },
    .lvl_term = (bool[]){ true, true, true },
};
```

#### B.4.2 页表操作

**页表项获取：**

```c
pte_t* pt_get_pte(struct page_table* pt, size_t lvl, vaddr_t va)
{
    size_t pte_index = pt_getpteindex_by_va(pt, va, 0);
    pte_t* pte = &(pt->root[pte_index]);
    
    for (size_t i = 0; i < lvl; i++) {
        if (!pte_valid(pte)) {
            return NULL;
        }
        pte = (pte_t*)pte_addr(pte);
        size_t index = pt_getpteindex_by_va(pt, va, i + 1);
        pte = &pte[index];
    }
    
    return pte;
}
```

**页表获取：**

```c
pte_t* pt_get(struct page_table* pt, size_t lvl, vaddr_t va)
{
    uintptr_t pte = (uintptr_t)pt_get_pte(pt, lvl, va);
    return (pte_t*)(pte & ~(pt_size(pt, lvl) - 1));
}
```

**页表项类型检查：**

```c
bool pte_page(struct page_table* pt, pte_t* pte, size_t lvl)
{
    UNUSED_ARG(pt);
    UNUSED_ARG(lvl);
    
    return ((*pte & PTE_VALID) != 0) && ((*pte & PTE_RWX) != 0);
}
```

**页表有效性检查：**

```c
bool pte_valid(pte_t* pte)
{
    return (*pte & PTE_VALID) != 0;
}
```

### B.5 RISC-V 架构总结

**关键技术特性：**

1. **H 扩展虚拟化**：
   - 完整的 Stage-2 地址转换
   - HGATP 寄存器配置
   - 异常委托机制
   - 虚拟 CSR 管理

2. **SBI 接口实现**：
   - 标准 SBI 扩展支持
   - Bao 特定 hypercall 扩展
   - Hart 启动/停止/挂起管理
   - IPI 发送机制
   - 远程内存屏障

3. **灵活的页表支持**：
   - Sv32/Sv39/Sv48 模式
   - RV32/RV64 支持
   - 多级页表遍历

4. **中断虚拟化**：
   - PLIC/IMSIC/APLIC 支持
   - 虚拟中断控制器
   - 定时器中断处理

5. **异常处理**：
   - 同步异常处理
   - 页错误模拟
   - SBI 调用处理

**设计优势：**

- **标准化**：遵循 RISC-V 规范
- **可扩展性**：模块化扩展支持
- **高效性**：直接硬件访问
- **兼容性**：支持多种页表模式
- **实时性**：确定性的异常处理路径
