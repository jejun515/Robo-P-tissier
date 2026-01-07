// PaymentModal.jsx
import React, { useState, useEffect } from "react";

export default function PaymentModal({ isOpen, onClose, onPaymentComplete, pickupData, price = 35000 }) {
  const [isProcessing, setIsProcessing] = useState(false);
  const [progress, setProgress] = useState(0);
  const [selectedMethod, setSelectedMethod] = useState("kakao");
  
  // 📞 [수정 1] 전화번호 상태 관리 추가
  const [phone, setPhone] = useState(""); 

  // 모달이 열릴 때마다 상태 초기화
  useEffect(() => {
    if (isOpen) {
      setIsProcessing(false);
      setProgress(0);
      setSelectedMethod("kakao");
      setPhone(""); // 📞 전화번호도 초기화
    }
  }, [isOpen]);

  if (!isOpen) return null;

  const handlePayment = () => {
    // 📞 [수정 2] 유효성 검사 (번호 미입력 시 차단)
    if (!phone || phone.length < 10) {
        alert("알림을 받을 휴대폰 번호를 입력해주세요!");
        return;
    }

    setIsProcessing(true);

    // 결제 진행 애니메이션 (2초)
    const interval = setInterval(() => {
      setProgress((prev) => {
        if (prev >= 100) {
          clearInterval(interval);
          return 100;
        }
        return prev + 5;
      });
    }, 50);

    // 2초 뒤 결제 완료 처리
    setTimeout(() => {
      clearInterval(interval);
      // 📞 [수정 3] 입력받은 phone 상태를 함께 전달
      onPaymentComplete({ 
          ...pickupData, 
          paymentMethod: selectedMethod, 
          phone: phone 
      }); 
    }, 2000);
  };

  return (
    <div className="fixed inset-0 bg-black/70 flex items-center justify-center z-[9999] backdrop-blur-md animate-in fade-in duration-300">
      <div className="bg-white w-full max-w-sm rounded-3xl p-8 shadow-2xl relative overflow-hidden">
        
        {/* 상단 헤더 */}
        <div className="text-center mb-6 border-b-2 border-dashed border-gray-200 pb-5">
          <div className="text-4xl mb-2">💳</div>
          <h2 className="text-2xl font-black text-slate-800">결제 요청</h2>
          <p className="text-gray-400 text-xs mt-1">결제 수단을 선택해주세요.</p>
        </div>

        {/* 금액 표시 */}
        <div className="flex justify-between items-center mb-6 px-4">
          <span className="text-slate-500 font-bold">총 결제금액</span>
          <span className="text-2xl font-black text-blue-600">{price.toLocaleString()}원</span>
        </div>

        {/* 📞 [수정 4] 연락처 입력 필드 (위치 조정 및 상태 연결) */}
        <div className="mb-6">
          <label className="block text-xs font-bold text-slate-500 mb-2">알림 받을 연락처</label>
          <input
            type="tel"
            value={phone}
            onChange={(e) => setPhone(e.target.value.replace(/[^0-9]/g, ''))} // 숫자만 입력되도록 처리
            placeholder="01012345678 (숫자만)"
            className="w-full p-3 bg-slate-50 border border-slate-200 rounded-xl font-bold text-slate-700 focus:outline-none focus:border-blue-500 transition"
            maxLength={11}
          />
        </div>

        {/* 💳 결제 수단 선택 */}
        <div className="flex flex-col gap-3 mb-8">
            <div className="grid grid-cols-2 gap-3">
                <button 
                    onClick={() => setSelectedMethod("kakao")}
                    className={`py-3 rounded-xl border flex flex-col items-center justify-center gap-1 transition-all ${
                        selectedMethod === "kakao" 
                        ? "bg-[#FEE500] border-[#FEE500] text-slate-900 ring-2 ring-offset-2 ring-[#FEE500] font-bold shadow-md" 
                        : "bg-white border-gray-200 text-slate-400 hover:bg-gray-50"
                    }`}
                >
                    <span className="text-xl">💬</span>
                    <span className="text-xs">카카오페이</span>
                </button>

                <button 
                    onClick={() => setSelectedMethod("naver")}
                    className={`py-3 rounded-xl border flex flex-col items-center justify-center gap-1 transition-all ${
                        selectedMethod === "naver" 
                        ? "bg-[#03C75A] border-[#03C75A] text-white ring-2 ring-offset-2 ring-[#03C75A] font-bold shadow-md" 
                        : "bg-white border-gray-200 text-slate-400 hover:bg-gray-50"
                    }`}
                >
                    <span className="text-xl">🇳</span>
                    <span className="text-xs">네이버페이</span>
                </button>
            </div>

            <button 
                onClick={() => setSelectedMethod("card")}
                className={`py-3 rounded-xl border flex items-center justify-center gap-2 transition-all ${
                    selectedMethod === "card" 
                    ? "bg-blue-50 border-blue-200 text-blue-700 ring-2 ring-offset-2 ring-blue-200 font-bold shadow-md" 
                    : "bg-white border-gray-200 text-slate-400 hover:bg-gray-50"
                }`}
            >
                <span>💳</span>
                <span className="text-sm font-medium">신용카드 / 일반결제</span>
            </button>
        </div>

        {/* 결제 진행 상태바 */}
        {isProcessing && (
            <div className="w-full bg-gray-200 rounded-full h-2.5 mb-6 overflow-hidden">
                <div 
                    className="bg-blue-600 h-2.5 rounded-full transition-all duration-100 ease-out" 
                    style={{ width: `${progress}%` }}
                ></div>
                <p className="text-center text-[10px] text-blue-500 mt-2 font-bold animate-pulse">
                    {selectedMethod === 'kakao' ? '카카오페이' : selectedMethod === 'naver' ? '네이버페이' : '카드사'} 승인 중...
                </p>
            </div>
        )}

        {/* 하단 버튼 영역 */}
        {!isProcessing && (
            <div className="grid grid-cols-2 gap-3">
            <button
                onClick={onClose}
                className="py-4 bg-gray-100 hover:bg-gray-200 text-gray-500 font-bold rounded-2xl transition"
            >
                취소
            </button>
            <button
                onClick={handlePayment}
                className="py-4 bg-slate-900 hover:bg-slate-800 text-white font-bold rounded-2xl shadow-lg transition transform active:scale-95"
            >
                {selectedMethod === 'kakao' ? '카카오 결제' : selectedMethod === 'naver' ? '네이버 결제' : '결제하기'}
            </button>
            </div>
        )}
      </div>
    </div>
  );
}