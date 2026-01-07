import React, { useState, useEffect, useRef } from "react";
import { ref, onValue, getDatabase } from "firebase/database";

// 💰 가격 설정
const CAKE_PRICES = {
  "1호": 25000,
  "2호": 35000,
};

// 🔢 단계별 순서 정의 (Rank) - 이 순서대로만 진행됨 (역주행 방지)
const STEP_ORDER = {
  pending: 0,
  start: 1,
  syrup: 2,
  powder: 3,
  topping_setup: 4, // 이 단계가 되면 이후 자잘한 토핑 동작은 무시됨
  pickup: 5,        // 픽업 단계
  done: 6           // 완료
};

// 📊 공정률 매핑 (UI 표시용)
const PROCESS_STEPS = {
  pending: { percent: 0, label: "대기중 (Idle)", color: "bg-slate-200", text: "text-slate-400" },
  start: { percent: 10, label: "작업 시작 (Start)", color: "bg-indigo-500", text: "text-indigo-600" },
  syrup: { percent: 30, label: "시럽 도포 (Syrup)", color: "bg-blue-500", text: "text-blue-600" },
  powder: { percent: 50, label: "파우더링 (Powder)", color: "bg-teal-500", text: "text-teal-600" },
  topping_setup: { percent: 70, label: "토핑 작업 (Topping)", color: "bg-orange-500", text: "text-orange-600" },
  pickup: { percent: 90, label: "픽업 대기 (Pickup)", color: "bg-purple-500", text: "text-purple-600" },
  done: { percent: 100, label: "완료 (Done)", color: "bg-green-500", text: "text-green-600" },
};

export default function Admin({ onLogout }) {
  const [orders, setOrders] = useState([]);
  
  // 현재 상태 관리
  const [currentStatus, setCurrentStatus] = useState("pending");
  
  // "완료 후 대기 중"인지 확인하는 플래그 (타이머 도는 동안 DB 무시용)
  const isResettingRef = useRef(false);

  const [stats, setStats] = useState({
    totalSales: 0,
    orderCount: 0,
    inventory: {
      choco_syrup: 0,
      strawberry_syrup: 0,
      sugar_powder: 0,
      choco_powder: 0,
    },
  });

  useEffect(() => {
    const db = getDatabase();
    
    // 1. 주문 내역 리스너 (기존 동일)
    const ordersRef = ref(db, "orders");
    const unsubscribeOrders = onValue(ordersRef, (snapshot) => {
      const data = snapshot.val();
      if (data) {
        const orderList = Object.entries(data)
          .map(([id, val]) => ({ id, ...val }))
          .reverse();
        setOrders(orderList);
        calculateStats(orderList);
      }
    });

    // 2. 현재 로봇 상태 리스너 (핵심 수정 부분)
    const statusRef = ref(db, "order_status");
    const unsubscribeStatus = onValue(statusRef, (snapshot) => {
      const dbValue = snapshot.val() || "pending"; // DB에서 온 값
      
      // 타이머가 돌고 있다면(완료 후 10초 대기 중) DB 값 무시
      if (isResettingRef.current) return;

      setCurrentStatus((prevStatus) => {
        const prevRank = STEP_ORDER[prevStatus] || 0;
        const newRank = STEP_ORDER[dbValue] || 0;

        // 🚀 핵심 로직:
        // 1. 새로운 단계(newRank)가 현재 단계(prevRank)보다 클 때만 업데이트 (진전이 있을 때만)
        // 2. 예외: DB값이 아예 모르는 값(rank 0)이 들어오면 무시됨 (topping_setup 유지)
        if (newRank > prevRank) {
          return dbValue;
        }
        
        // 그 외(과거 단계거나, 알 수 없는 단계)는 현재 상태 유지
        return prevStatus;
      });
    });

    return () => {
      unsubscribeOrders();
      unsubscribeStatus();
    };
  }, []);

  // 3. 완료(Done) 상태 감지 및 10초 뒤 초기화 타이머
  useEffect(() => {
    if (currentStatus === 'done') {
      console.log("🎉 작업 완료! 10초 뒤 초기화됩니다.");
      isResettingRef.current = true; // DB 업데이트 차단 시작

      const timer = setTimeout(() => {
        setCurrentStatus('pending'); // 0%로 초기화
        isResettingRef.current = false; // DB 업데이트 차단 해제
        console.log("🔄 상태 초기화 완료 (Pending)");
      }, 10000); // 10초 (10000ms)

      return () => clearTimeout(timer);
    }
  }, [currentStatus]);


  const calculateStats = (orderList) => {
    let sales = 0;
    const inv = {
      choco_syrup: 0,
      strawberry_syrup: 0,
      sugar_powder: 0,
      choco_powder: 0,
    };

    orderList.forEach((order) => {
      const price = CAKE_PRICES[order.size] || 0;
      sales += price;
      if (order.syrup && inv.hasOwnProperty(order.syrup)) inv[order.syrup]++;
      if (order.powder && inv.hasOwnProperty(order.powder)) inv[order.powder]++;
    });

    setStats({
      totalSales: sales,
      orderCount: orderList.length,
      inventory: inv,
    });
  };

  const getCurrentProcessInfo = () => {
    // 정의되지 않은 상태가 들어와도 기존 상태 유지되므로 안전하지만, 예외처리 추가
    return PROCESS_STEPS[currentStatus] || PROCESS_STEPS["pending"];
  };

  const currentProcess = getCurrentProcessInfo();

  return (
    <div className="min-h-screen bg-slate-50 text-slate-900 font-sans flex flex-col animate-in fade-in duration-500">
      {/* 헤더 */}
      <header className="bg-white border-b border-slate-200 p-6 flex justify-between items-center shadow-sm sticky top-0 z-20">
        <div className="flex items-center gap-3">
          <div className="p-2 bg-indigo-600 rounded-xl text-white text-xl">📊</div>
          <div>
            <h1 className="text-xl font-black text-slate-800">매장 관리 시스템</h1>
            <p className="text-xs text-slate-400 font-bold uppercase tracking-tight">
              ROKEY Robot Dashboard
            </p>
          </div>
        </div>
        <button
          onClick={onLogout}
          className="px-5 py-2 bg-slate-100 hover:bg-rose-50 hover:text-rose-600 text-slate-600 rounded-xl text-sm font-bold transition-all border border-slate-200 shadow-sm active:scale-95"
        >
          로그아웃
        </button>
      </header>

      <main className="p-8 max-w-7xl mx-auto w-full space-y-8">
        
        {/* 공정률 대시보드 */}
        <section className="bg-white rounded-3xl shadow-lg border border-indigo-100 p-8 relative overflow-hidden">
          <div className="absolute top-0 right-0 p-4 opacity-10 pointer-events-none">
             <span className="text-9xl">🤖</span>
          </div>

          <div className="flex flex-col md:flex-row justify-between items-end mb-6 relative z-10">
            <div>
              <span className="inline-flex items-center gap-2 px-3 py-1 rounded-full bg-indigo-50 text-indigo-600 text-[10px] font-black uppercase tracking-wider mb-2">
                <span className={`w-2 h-2 rounded-full bg-indigo-600 ${currentStatus !== 'done' ? 'animate-pulse' : ''}`}></span>
                Robot Live Status
              </span>
              <h2 className="text-2xl font-black text-slate-800">현재 케이크 공정률</h2>
              <p className="text-sm text-slate-500 mt-1 font-medium">
                {currentStatus === 'done' 
                  ? "작업이 완료되었습니다. 잠시 후 대기 상태로 전환됩니다." 
                  : "로봇이 현재 수행 중인 작업을 실시간으로 모니터링합니다."}
              </p>
            </div>
            <div className="text-right mt-4 md:mt-0">
               <p className="text-xs font-bold text-slate-400 uppercase">Current Step</p>
               <p className={`text-3xl font-black ${currentProcess.text} transition-colors duration-300`}>
                 {currentProcess.label}
               </p>
            </div>
          </div>

          {/* 게이지 바 */}
          <div className="relative h-8 bg-slate-100 rounded-full overflow-hidden shadow-inner border border-slate-200">
             <div className="absolute inset-0 flex justify-between px-2 items-center z-10 pointer-events-none">
                {[20, 40, 60, 80].map(pt => (
                   <div key={pt} className="h-full w-[1px] bg-white/50"></div>
                ))}
             </div>
             <div 
               className={`h-full rounded-full transition-all duration-1000 ease-in-out flex items-center justify-end pr-3 shadow-lg relative ${currentProcess.color} ${currentStatus !== 'done' ? 'animate-pulse-slow' : ''}`}
               style={{ width: `${currentProcess.percent}%` }}
             >
                <span className="text-white font-bold text-sm drop-shadow-md">
                  {currentProcess.percent}%
                </span>
             </div>
          </div>

          {/* 하단 단계 아이콘 */}
          <div className="flex justify-between mt-3 px-1">
             {Object.entries(PROCESS_STEPS).map(([key, step]) => (
                key !== 'pending' && (
                  <div key={key} className={`flex flex-col items-center transition-all duration-500 ${currentProcess.percent >= step.percent ? 'opacity-100 transform scale-105' : 'opacity-30 grayscale'}`}>
                     <div className={`w-3 h-3 rounded-full mb-1 ${step.color}`}></div>
                     <span className="text-[10px] font-bold uppercase text-slate-500">{key.replace('_', ' ')}</span>
                  </div>
                )
             ))}
          </div>
        </section>

        {/* 하단 통계 및 테이블 (기존 동일) */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          <div className="bg-white p-6 rounded-3xl shadow-sm border border-slate-100">
            <p className="text-slate-400 text-xs font-bold uppercase mb-2">총 누적 매출</p>
            <p className="text-3xl font-black text-indigo-600">₩ {stats.totalSales.toLocaleString()}</p>
          </div>
          <div className="bg-white p-6 rounded-3xl shadow-sm border border-slate-100">
            <p className="text-slate-400 text-xs font-bold uppercase mb-2">총 주문 건수</p>
            <p className="text-3xl font-black text-slate-800">{stats.orderCount} <span className="text-lg font-medium text-slate-400">건</span></p>
          </div>
          <div className="bg-white p-6 rounded-3xl shadow-sm border border-slate-100">
            <p className="text-slate-400 text-xs font-bold uppercase mb-2">평균 주문 금액</p>
            <p className="text-3xl font-black text-slate-800">
              ₩ {stats.orderCount > 0 ? Math.floor(stats.totalSales / stats.orderCount).toLocaleString() : 0}
            </p>
          </div>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-12 gap-8">
          <div className="lg:col-span-8 bg-white rounded-3xl shadow-sm border border-slate-100 overflow-hidden">
            <div className="p-6 border-b border-slate-50 flex justify-between items-center bg-white">
              <h2 className="font-black text-slate-700 text-lg">최근 주문 접수 현황</h2>
            </div>
            <div className="overflow-x-auto max-h-[600px]">
              <table className="w-full text-left">
                <thead className="bg-slate-50 text-slate-400 text-[10px] uppercase font-bold sticky top-0">
                  <tr>
                    <th className="px-6 py-4">주문번호</th>
                    <th className="px-6 py-4">도안 타입</th>
                    <th className="px-6 py-4">사이즈</th>
                    <th className="px-6 py-4">결제 금액</th>
                    <th className="px-6 py-4">상태</th>
                  </tr>
                </thead>
                <tbody className="divide-y divide-slate-50">
                  {orders.length > 0 ? (
                    orders.map((order) => (
                      <tr key={order.id} className="hover:bg-slate-50/50 transition-colors">
                        <td className="px-6 py-4 font-mono text-sm text-slate-500">#{order.id.toString().slice(-5)}</td>
                        <td className="px-6 py-4 font-bold text-slate-700">{order.type || '기본'}</td>
                        <td className="px-6 py-4"><span className="px-2 py-1 bg-slate-100 rounded-md text-xs font-semibold">{order.size}</span></td>
                        <td className="px-6 py-4 font-bold text-indigo-600 text-sm">₩ {(CAKE_PRICES[order.size] || 0).toLocaleString()}</td>
                        <td className="px-6 py-4">
                           <span className={`px-3 py-1 rounded-full text-[10px] font-black uppercase tracking-wide border ${
                             order.status === 'done' ? 'bg-green-50 text-green-600 border-green-100' : 
                             'bg-slate-50 text-slate-500 border-slate-100'
                           }`}>
                             {order.status || 'Waiting'}
                           </span>
                        </td>
                      </tr>
                    ))
                  ) : (
                    <tr>
                      <td colSpan="5" className="px-6 py-10 text-center text-slate-400 font-medium italic">
                        현재 접수된 주문이 없습니다.
                      </td>
                    </tr>
                  )}
                </tbody>
              </table>
            </div>
          </div>

          <div className="lg:col-span-4 space-y-6">
            <div className="bg-white p-6 rounded-3xl shadow-sm border border-slate-100">
              <h2 className="font-black text-slate-700 text-lg mb-6">소모품 사용 통계</h2>
              <div className="space-y-6">
                {Object.entries(stats.inventory).map(([item, count]) => (
                  <div key={item}>
                    <div className="flex justify-between text-sm mb-2">
                      <span className="font-bold text-slate-500 uppercase tracking-tighter">
                        {item.replace('_', ' ')}
                      </span>
                      <span className="font-black text-slate-800">
                        {count} <small className="text-slate-400">회</small>
                      </span>
                    </div>
                    <div className="w-full h-2.5 bg-slate-100 rounded-full overflow-hidden">
                      <div 
                        className={`h-full rounded-full transition-all duration-1000 ${
                          count > 40 ? 'bg-rose-500' : count > 25 ? 'bg-amber-500' : 'bg-indigo-500'
                        }`}
                        style={{ width: `${Math.min((count / 50) * 100, 100)}%` }}
                      ></div>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </main>
      
      <footer className="p-6 text-center text-slate-400 text-xs font-medium">
        &copy; {new Date().getFullYear()} ROKEY Cake Robot System.
      </footer>
    </div>
  );
}