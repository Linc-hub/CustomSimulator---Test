(module
 (type $0 (func))
 (type $1 (func (param i32 i32 i32 i32)))
 (type $2 (func (param i32) (result i32)))
 (type $3 (func (param i32 i32 f64)))
 (type $4 (func (param i32)))
 (type $5 (func (param i32 i32) (result f64)))
 (type $6 (func (param i32) (result f64)))
 (type $7 (func (param i32 i32 i32)))
 (type $8 (func (param i32 i32)))
 (type $9 (func (param i32 f64 i32 i32)))
 (import "env" "abort" (func $~lib/builtins/abort (param i32 i32 i32 i32)))
 (global $~lib/rt/stub/offset (mut i32) (i32.const 0))
 (global $wasm/jacobian/jacobian (mut i32) (i32.const 0))
 (global $wasm/jacobian/jtj (mut i32) (i32.const 0))
 (global $wasm/jacobian/vectorA (mut i32) (i32.const 0))
 (global $wasm/jacobian/vectorB (mut i32) (i32.const 0))
 (global $wasm/jacobian/vectorC (mut i32) (i32.const 0))
 (global $wasm/jacobian/output (mut i32) (i32.const 0))
 (global $wasm/jacobian/jacobianPtr (mut i32) (i32.const 0))
 (global $wasm/jacobian/resultPtr (mut i32) (i32.const 0))
 (memory $0 1)
 (data $0 (i32.const 1036) ",")
 (data $0.1 (i32.const 1048) "\02\00\00\00\1c\00\00\00I\00n\00v\00a\00l\00i\00d\00 \00l\00e\00n\00g\00t\00h")
 (data $1 (i32.const 1084) "<")
 (data $1.1 (i32.const 1096) "\02\00\00\00&\00\00\00~\00l\00i\00b\00/\00s\00t\00a\00t\00i\00c\00a\00r\00r\00a\00y\00.\00t\00s")
 (data $2 (i32.const 1148) "<")
 (data $2.1 (i32.const 1160) "\02\00\00\00(\00\00\00A\00l\00l\00o\00c\00a\00t\00i\00o\00n\00 \00t\00o\00o\00 \00l\00a\00r\00g\00e")
 (data $3 (i32.const 1212) "<")
 (data $3.1 (i32.const 1224) "\02\00\00\00\1e\00\00\00~\00l\00i\00b\00/\00r\00t\00/\00s\00t\00u\00b\00.\00t\00s")
 (data $4 (i32.const 1276) "<")
 (data $4.1 (i32.const 1288) "\02\00\00\00$\00\00\00I\00n\00d\00e\00x\00 \00o\00u\00t\00 \00o\00f\00 \00r\00a\00n\00g\00e")
 (export "jacobianPtr" (global $wasm/jacobian/jacobianPtr))
 (export "resultPtr" (global $wasm/jacobian/resultPtr))
 (export "evaluate" (func $wasm/jacobian/evaluate))
 (export "memory" (memory $0))
 (start $~start)
 (func $~lib/staticarray/StaticArray<f64>#constructor (param $0 i32) (result i32)
  (local $1 i32)
  (local $2 i32)
  (local $3 i32)
  (local $4 i32)
  (local $5 i32)
  (local $6 i32)
  local.get $0
  i32.const 134217727
  i32.gt_u
  if
   i32.const 1056
   i32.const 1104
   i32.const 51
   i32.const 60
   call $~lib/builtins/abort
   unreachable
  end
  local.get $0
  i32.const 3
  i32.shl
  local.tee $2
  i32.const 1073741804
  i32.gt_u
  if
   i32.const 1168
   i32.const 1232
   i32.const 86
   i32.const 30
   call $~lib/builtins/abort
   unreachable
  end
  local.get $2
  i32.const 16
  i32.add
  local.tee $1
  i32.const 1073741820
  i32.gt_u
  if
   i32.const 1168
   i32.const 1232
   i32.const 33
   i32.const 29
   call $~lib/builtins/abort
   unreachable
  end
  global.get $~lib/rt/stub/offset
  global.get $~lib/rt/stub/offset
  i32.const 4
  i32.add
  local.tee $4
  local.get $1
  i32.const 19
  i32.add
  i32.const -16
  i32.and
  i32.const 4
  i32.sub
  local.tee $1
  i32.add
  local.tee $3
  memory.size
  local.tee $5
  i32.const 16
  i32.shl
  i32.const 15
  i32.add
  i32.const -16
  i32.and
  local.tee $6
  i32.gt_u
  if
   local.get $5
   local.get $3
   local.get $6
   i32.sub
   i32.const 65535
   i32.add
   i32.const -65536
   i32.and
   i32.const 16
   i32.shr_u
   local.tee $6
   local.get $5
   local.get $6
   i32.gt_s
   select
   memory.grow
   i32.const 0
   i32.lt_s
   if
    local.get $6
    memory.grow
    i32.const 0
    i32.lt_s
    if
     unreachable
    end
   end
  end
  local.get $3
  global.set $~lib/rt/stub/offset
  local.get $1
  i32.store
  local.get $4
  i32.const 4
  i32.sub
  local.tee $0
  i32.const 0
  i32.store offset=4
  local.get $0
  i32.const 0
  i32.store offset=8
  local.get $0
  i32.const 4
  i32.store offset=12
  local.get $0
  local.get $2
  i32.store offset=16
  local.get $4
  i32.const 16
  i32.add
  local.tee $0
  i32.const 0
  local.get $2
  memory.fill
  local.get $0
 )
 (func $~lib/staticarray/StaticArray<f64>#__set (param $0 i32) (param $1 i32) (param $2 f64)
  local.get $1
  local.get $0
  i32.const 20
  i32.sub
  i32.load offset=16
  i32.const 3
  i32.shr_u
  i32.ge_u
  if
   i32.const 1296
   i32.const 1104
   i32.const 93
   i32.const 41
   call $~lib/builtins/abort
   unreachable
  end
  local.get $0
  local.get $1
  i32.const 3
  i32.shl
  i32.add
  local.get $2
  f64.store
 )
 (func $wasm/jacobian/zeroArray (param $0 i32)
  (local $1 i32)
  loop $for-loop|0
   local.get $1
   local.get $0
   i32.const 20
   i32.sub
   i32.load offset=16
   i32.const 3
   i32.shr_u
   i32.lt_s
   if
    local.get $0
    local.get $1
    f64.const 0
    call $~lib/staticarray/StaticArray<f64>#__set
    local.get $1
    i32.const 1
    i32.add
    local.set $1
    br $for-loop|0
   end
  end
 )
 (func $~lib/staticarray/StaticArray<f64>#__get (param $0 i32) (param $1 i32) (result f64)
  local.get $1
  local.get $0
  i32.const 20
  i32.sub
  i32.load offset=16
  i32.const 3
  i32.shr_u
  i32.ge_u
  if
   i32.const 1296
   i32.const 1104
   i32.const 78
   i32.const 41
   call $~lib/builtins/abort
   unreachable
  end
  local.get $0
  local.get $1
  i32.const 3
  i32.shl
  i32.add
  f64.load
 )
 (func $wasm/jacobian/normalizeVector (param $0 i32) (result f64)
  (local $1 i32)
  (local $2 f64)
  (local $3 f64)
  loop $for-loop|0
   local.get $1
   local.get $0
   i32.const 20
   i32.sub
   i32.load offset=16
   i32.const 3
   i32.shr_u
   i32.lt_s
   if
    local.get $2
    local.get $0
    local.get $1
    call $~lib/staticarray/StaticArray<f64>#__get
    local.tee $2
    local.get $2
    f64.mul
    f64.add
    local.set $2
    local.get $1
    i32.const 1
    i32.add
    local.set $1
    br $for-loop|0
   end
  end
  local.get $2
  f64.sqrt
  local.tee $3
  f64.const 1e-12
  f64.lt
  if
   f64.const 0
   return
  end
  f64.const 1
  local.get $3
  f64.div
  local.set $2
  i32.const 0
  local.set $1
  loop $for-loop|1
   local.get $1
   local.get $0
   i32.const 20
   i32.sub
   i32.load offset=16
   i32.const 3
   i32.shr_u
   i32.lt_s
   if
    local.get $0
    local.get $1
    local.get $0
    local.get $1
    call $~lib/staticarray/StaticArray<f64>#__get
    local.get $2
    f64.mul
    call $~lib/staticarray/StaticArray<f64>#__set
    local.get $1
    i32.const 1
    i32.add
    local.set $1
    br $for-loop|1
   end
  end
  local.get $3
 )
 (func $wasm/jacobian/multiply (param $0 i32) (param $1 i32) (param $2 i32)
  (local $3 i32)
  (local $4 i32)
  (local $5 f64)
  loop $for-loop|0
   local.get $4
   i32.const 6
   i32.lt_s
   if
    f64.const 0
    local.set $5
    i32.const 0
    local.set $3
    loop $for-loop|1
     local.get $3
     i32.const 6
     i32.lt_s
     if
      local.get $5
      local.get $0
      local.get $4
      i32.const 6
      i32.mul
      local.get $3
      i32.add
      call $~lib/staticarray/StaticArray<f64>#__get
      local.get $1
      local.get $3
      call $~lib/staticarray/StaticArray<f64>#__get
      f64.mul
      f64.add
      local.set $5
      local.get $3
      i32.const 1
      i32.add
      local.set $3
      br $for-loop|1
     end
    end
    local.get $2
    local.get $4
    local.get $5
    call $~lib/staticarray/StaticArray<f64>#__set
    local.get $4
    i32.const 1
    i32.add
    local.set $4
    br $for-loop|0
   end
  end
 )
 (func $wasm/jacobian/copyArray (param $0 i32) (param $1 i32)
  (local $2 i32)
  loop $for-loop|0
   local.get $2
   local.get $0
   i32.const 20
   i32.sub
   i32.load offset=16
   i32.const 3
   i32.shr_u
   i32.lt_s
   if
    local.get $1
    local.get $2
    local.get $0
    local.get $2
    call $~lib/staticarray/StaticArray<f64>#__get
    call $~lib/staticarray/StaticArray<f64>#__set
    local.get $2
    i32.const 1
    i32.add
    local.set $2
    br $for-loop|0
   end
  end
 )
 (func $wasm/jacobian/multiplyShifted (param $0 i32) (param $1 f64) (param $2 i32) (param $3 i32)
  (local $4 i32)
  (local $5 i32)
  (local $6 f64)
  loop $for-loop|0
   local.get $4
   i32.const 6
   i32.lt_s
   if
    f64.const 0
    local.set $6
    i32.const 0
    local.set $5
    loop $for-loop|1
     local.get $5
     i32.const 6
     i32.lt_s
     if
      local.get $6
      local.get $0
      local.get $4
      i32.const 6
      i32.mul
      local.get $5
      i32.add
      call $~lib/staticarray/StaticArray<f64>#__get
      local.get $2
      local.get $5
      call $~lib/staticarray/StaticArray<f64>#__get
      f64.mul
      f64.add
      local.set $6
      local.get $5
      i32.const 1
      i32.add
      local.set $5
      br $for-loop|1
     end
    end
    local.get $3
    local.get $4
    local.get $1
    local.get $2
    local.get $4
    call $~lib/staticarray/StaticArray<f64>#__get
    f64.mul
    local.get $6
    f64.sub
    call $~lib/staticarray/StaticArray<f64>#__set
    local.get $4
    i32.const 1
    i32.add
    local.set $4
    br $for-loop|0
   end
  end
 )
 (func $wasm/jacobian/evaluate
  (local $0 i32)
  (local $1 i32)
  (local $2 f64)
  (local $3 f64)
  (local $4 i32)
  (local $5 f64)
  (local $6 i32)
  (local $7 i32)
  loop $for-loop|0
   local.get $1
   i32.const 6
   i32.lt_s
   if
    local.get $1
    local.set $0
    loop $for-loop|1
     local.get $0
     i32.const 6
     i32.lt_s
     if
      f64.const 0
      local.set $2
      i32.const 0
      local.set $4
      loop $for-loop|2
       local.get $4
       i32.const 6
       i32.lt_s
       if
        local.get $2
        global.get $wasm/jacobian/jacobianPtr
        local.get $4
        i32.const 6
        i32.mul
        local.tee $6
        local.get $1
        i32.add
        i32.const 3
        i32.shl
        i32.add
        f64.load
        global.get $wasm/jacobian/jacobianPtr
        local.get $0
        local.get $6
        i32.add
        i32.const 3
        i32.shl
        i32.add
        f64.load
        f64.mul
        f64.add
        local.set $2
        local.get $4
        i32.const 1
        i32.add
        local.set $4
        br $for-loop|2
       end
      end
      global.get $wasm/jacobian/jtj
      local.get $1
      i32.const 6
      i32.mul
      local.get $0
      i32.add
      local.get $2
      call $~lib/staticarray/StaticArray<f64>#__set
      global.get $wasm/jacobian/jtj
      local.get $0
      i32.const 6
      i32.mul
      local.get $1
      i32.add
      local.get $2
      call $~lib/staticarray/StaticArray<f64>#__set
      local.get $0
      i32.const 1
      i32.add
      local.set $0
      br $for-loop|1
     end
    end
    local.get $1
    i32.const 1
    i32.add
    local.set $1
    br $for-loop|0
   end
  end
  global.get $wasm/jacobian/vectorA
  local.set $1
  i32.const 0
  local.set $0
  loop $for-loop|00
   local.get $0
   local.get $1
   i32.const 20
   i32.sub
   i32.load offset=16
   i32.const 3
   i32.shr_u
   i32.lt_s
   if
    local.get $1
    local.get $0
    f64.const 1
    call $~lib/staticarray/StaticArray<f64>#__set
    local.get $0
    i32.const 1
    i32.add
    local.set $0
    br $for-loop|00
   end
  end
  global.get $wasm/jacobian/vectorB
  call $wasm/jacobian/zeroArray
  global.get $wasm/jacobian/vectorC
  call $wasm/jacobian/zeroArray
  global.get $wasm/jacobian/jtj
  local.set $0
  global.get $wasm/jacobian/vectorB
  local.set $1
  global.get $wasm/jacobian/vectorC
  local.set $6
  global.get $wasm/jacobian/vectorA
  local.tee $7
  call $wasm/jacobian/normalizeVector
  f64.const 0
  f64.eq
  if
   local.get $7
   i32.const 0
   f64.const 1
   call $~lib/staticarray/StaticArray<f64>#__set
   local.get $7
   call $wasm/jacobian/normalizeVector
   drop
  end
  i32.const 0
  local.set $4
  loop $for-loop|01
   local.get $4
   i32.const 18
   i32.lt_s
   if
    block $for-break0
     local.get $0
     local.get $7
     local.get $1
     call $wasm/jacobian/multiply
     local.get $1
     call $wasm/jacobian/normalizeVector
     f64.const 0
     f64.eq
     br_if $for-break0
     local.get $1
     local.get $7
     call $wasm/jacobian/copyArray
     local.get $4
     i32.const 1
     i32.add
     local.set $4
     br $for-loop|01
    end
   end
  end
  local.get $0
  local.get $7
  local.get $6
  call $wasm/jacobian/multiply
  f64.const 0
  local.set $2
  i32.const 0
  local.set $4
  loop $for-loop|002
   local.get $4
   local.get $7
   i32.const 20
   i32.sub
   i32.load offset=16
   i32.const 3
   i32.shr_u
   i32.lt_s
   if
    local.get $2
    local.get $7
    local.get $4
    call $~lib/staticarray/StaticArray<f64>#__get
    local.get $6
    local.get $4
    call $~lib/staticarray/StaticArray<f64>#__get
    f64.mul
    f64.add
    local.set $2
    local.get $4
    i32.const 1
    i32.add
    local.set $4
    br $for-loop|002
   end
  end
  local.get $2
  local.tee $3
  f64.const 1e-12
  f64.le
  if
   global.get $wasm/jacobian/output
   i32.const 0
   f64.const 0
   call $~lib/staticarray/StaticArray<f64>#__set
   global.get $wasm/jacobian/output
   i32.const 1
   f64.const 0
   call $~lib/staticarray/StaticArray<f64>#__set
   return
  end
  global.get $wasm/jacobian/vectorA
  local.set $1
  i32.const 0
  local.set $0
  loop $for-loop|014
   local.get $0
   local.get $1
   i32.const 20
   i32.sub
   i32.load offset=16
   i32.const 3
   i32.shr_u
   i32.lt_s
   if
    local.get $1
    local.get $0
    f64.const 1
    local.get $0
    f64.convert_i32_s
    f64.const 0.15
    f64.mul
    f64.sub
    call $~lib/staticarray/StaticArray<f64>#__set
    local.get $0
    i32.const 1
    i32.add
    local.set $0
    br $for-loop|014
   end
  end
  global.get $wasm/jacobian/vectorB
  call $wasm/jacobian/zeroArray
  global.get $wasm/jacobian/vectorC
  call $wasm/jacobian/zeroArray
  global.get $wasm/jacobian/jtj
  local.set $0
  global.get $wasm/jacobian/vectorB
  local.set $1
  global.get $wasm/jacobian/vectorC
  local.set $6
  global.get $wasm/jacobian/vectorA
  local.tee $7
  call $wasm/jacobian/normalizeVector
  f64.const 0
  f64.eq
  if
   local.get $7
   i32.const 0
   f64.const 1
   call $~lib/staticarray/StaticArray<f64>#__set
   local.get $7
   call $wasm/jacobian/normalizeVector
   drop
  end
  local.get $3
  local.get $3
  f64.const 0.05
  f64.mul
  f64.const 1e-06
  f64.max
  f64.add
  local.set $5
  i32.const 0
  local.set $4
  loop $for-loop|06
   local.get $4
   i32.const 18
   i32.lt_s
   if
    block $for-break05
     local.get $0
     local.get $5
     local.get $7
     local.get $1
     call $wasm/jacobian/multiplyShifted
     local.get $1
     call $wasm/jacobian/normalizeVector
     f64.const 0
     f64.eq
     br_if $for-break05
     local.get $1
     local.get $7
     call $wasm/jacobian/copyArray
     local.get $4
     i32.const 1
     i32.add
     local.set $4
     br $for-loop|06
    end
   end
  end
  local.get $0
  local.get $5
  local.get $7
  local.get $6
  call $wasm/jacobian/multiplyShifted
  f64.const 0
  local.set $2
  i32.const 0
  local.set $4
  loop $for-loop|007
   local.get $4
   local.get $7
   i32.const 20
   i32.sub
   i32.load offset=16
   i32.const 3
   i32.shr_u
   i32.lt_s
   if
    local.get $2
    local.get $7
    local.get $4
    call $~lib/staticarray/StaticArray<f64>#__get
    local.get $6
    local.get $4
    call $~lib/staticarray/StaticArray<f64>#__get
    f64.mul
    f64.add
    local.set $2
    local.get $4
    i32.const 1
    i32.add
    local.set $4
    br $for-loop|007
   end
  end
  global.get $wasm/jacobian/output
  i32.const 0
  local.get $5
  local.get $2
  f64.sub
  local.tee $2
  f64.const 0
  f64.lt
  if (result f64)
   f64.const 0
  else
   local.get $2
  end
  f64.sqrt
  call $~lib/staticarray/StaticArray<f64>#__set
  global.get $wasm/jacobian/output
  i32.const 1
  local.get $3
  f64.sqrt
  call $~lib/staticarray/StaticArray<f64>#__set
 )
 (func $~start
  i32.const 1340
  global.set $~lib/rt/stub/offset
  i32.const 36
  call $~lib/staticarray/StaticArray<f64>#constructor
  global.set $wasm/jacobian/jacobian
  i32.const 36
  call $~lib/staticarray/StaticArray<f64>#constructor
  global.set $wasm/jacobian/jtj
  i32.const 6
  call $~lib/staticarray/StaticArray<f64>#constructor
  global.set $wasm/jacobian/vectorA
  i32.const 6
  call $~lib/staticarray/StaticArray<f64>#constructor
  global.set $wasm/jacobian/vectorB
  i32.const 6
  call $~lib/staticarray/StaticArray<f64>#constructor
  global.set $wasm/jacobian/vectorC
  i32.const 2
  call $~lib/staticarray/StaticArray<f64>#constructor
  global.set $wasm/jacobian/output
  global.get $wasm/jacobian/jacobian
  global.set $wasm/jacobian/jacobianPtr
  global.get $wasm/jacobian/output
  global.set $wasm/jacobian/resultPtr
 )
)
