import React, { useState, useRef, useEffect } from "react";

export default function PhotoUploader({ cakeSize, onUploadSuccess, onLog }) {
  const [selectedFile, setSelectedFile] = useState(null);
  const [preview, setPreview] = useState(null);
  const [isProcessing, setIsProcessing] = useState(false);
  
  // 편집 단계: 'upload' (파일선택) -> 'editing' (그리기) -> 'done' (완료)
  const [step, setStep] = useState('upload'); 

  // 경로 데이터 (정규화된 좌표 -1.0 ~ 1.0)
  const [autoPaths, setAutoPaths] = useState([]);   // 서버에서 받은 초록색 경로
  const [manualPaths, setManualPaths] = useState([]); // 내가 그린 빨간색 경로

  // Canvas 관련
  const canvasRef = useRef(null);
  const [isDrawing, setIsDrawing] = useState(false);
  const [currentStroke, setCurrentStroke] = useState([]);

  // 1. 파일 선택 핸들러
  const handleFileChange = (e) => {
    const file = e.target.files[0];
    if (file) {
      setSelectedFile(file);
      const url = URL.createObjectURL(file);
      setPreview(url);
      setStep('upload');
      setAutoPaths([]);
      setManualPaths([]);
      if(onLog) onLog(`🖼️ 파일 선택됨: ${file.name}`);
    }
  };

  // 2. [API] 이미지 분석 요청 (자동 경로 추출)
  const startEditing = async () => {
    if (!selectedFile) return;
    setIsProcessing(true);
    if(onLog) onLog("⚙️ 이미지 분석 중...");

    const formData = new FormData();
    formData.append("file", selectedFile);

    try {
        // 백엔드의 분석 API 호출
        const res = await fetch('http://127.0.0.1:8000/api/analyze_image', {
            method: 'POST', body: formData
        });
        const data = await res.json();
        
        if (data.status === 'success') {
            setAutoPaths(data.paths); // 서버가 준 경로 데이터
            setStep('editing');
            if(onLog) onLog(`✅ 분석 완료! 경로 ${data.paths.length}개 추출됨.`);
        } else {
            alert("분석 실패: " + data.message);
        }
    } catch (e) {
        console.error(e);
        alert("서버 통신 오류 (백엔드가 켜져있나요?)");
    } finally {
        setIsProcessing(false);
    }
  };

  // 3. Canvas 렌더링 (이미지 + 경로 그리기)
  useEffect(() => {
    if (step === 'editing' && canvasRef.current && preview) {
        const canvas = canvasRef.current;
        const ctx = canvas.getContext('2d');
        const img = new Image();
        img.src = preview;
        
        img.onload = () => {
            // 캔버스 크기 고정 (300x300)
            const W = 300;
            const H = 300;
            canvas.width = W;
            canvas.height = H;

            // 배경 이미지 그리기
            ctx.drawImage(img, 0, 0, W, H);

            // 좌표 변환 함수 (정규좌표 -1~1 -> 캔버스좌표 0~300)
            // 백엔드에서 온 정규좌표는 중앙이 (0,0)임
            const toCanvas = (u, v) => ({
                x: (u * (W/2)) + (W/2),
                y: (v * (H/2)) + (H/2)
            });

            // A. 자동 경로 그리기 (초록색)
            ctx.lineWidth = 2;
            ctx.strokeStyle = "#00ff00"; 
            autoPaths.forEach(path => {
                ctx.beginPath();
                path.forEach((pt, i) => {
                    const {x, y} = toCanvas(pt[0], pt[1]);
                    if(i===0) ctx.moveTo(x, y);
                    else ctx.lineTo(x, y);
                });
                ctx.stroke();
            });

            // B. 수동 경로 그리기 (빨간색)
            ctx.strokeStyle = "#ff0000"; 
            ctx.lineWidth = 3;
            // 이미 확정된 경로들 + 현재 그리고 있는 경로
            [...manualPaths, currentStroke].forEach(path => {
                if(path.length < 1) return;
                ctx.beginPath();
                path.forEach((pt, i) => {
                    const {x, y} = toCanvas(pt[0], pt[1]);
                    if(i===0) ctx.moveTo(x, y);
                    else ctx.lineTo(x, y);
                });
                ctx.stroke();
            });
        };
    }
  }, [step, preview, autoPaths, manualPaths, currentStroke]);

  // --- 마우스 이벤트 핸들러 (그리기 로직) ---
  const getNormXY = (e) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    // 픽셀(0~300) -> 정규 좌표(-1.0 ~ 1.0) 변환
    const u = (x - 150) / 150;
    const v = (y - 150) / 150;
    return [u, v];
  };

  const onMouseDown = (e) => {
    if(step !== 'editing') return;
    setIsDrawing(true);
    setCurrentStroke([getNormXY(e)]);
  };

  const onMouseMove = (e) => {
    if (!isDrawing) return;
    setCurrentStroke(prev => [...prev, getNormXY(e)]);
  };

  const onMouseUp = () => {
    if (!isDrawing) return;
    setIsDrawing(false);
    if (currentStroke.length > 2) {
        setManualPaths(prev => [...prev, currentStroke]);
    }
    setCurrentStroke([]);
  };

  // 4. [API] 최종 저장 (자동 + 수동 경로 합치기)
  const handleSave = async () => {
    if(onLog) onLog("💾 최종 경로 저장 중...");
    setIsProcessing(true);

    // 두 경로 합치기
    const finalPaths = [...autoPaths, ...manualPaths];

    try {
        const res = await fetch('http://127.0.0.1:8000/api/save_custom_paths', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                paths: finalPaths,
                size: cakeSize
            })
        });
        const result = await res.json();
        
        if (result.status === 'success') {
            if(onLog) onLog(`✅ 저장 완료! ID: ${result.design_id}`);
            // 부모 컴포넌트(App.jsx)에 design_id 전달
            if(onUploadSuccess) onUploadSuccess(result, preview);
            setStep('done');
        } else {
            alert("저장 실패: " + result.message);
        }
    } catch(e) {
        console.error(e);
        alert("서버 통신 오류");
    } finally {
        setIsProcessing(false);
    }
  };

  // 실행 취소
  const undoManual = () => {
      setManualPaths(prev => prev.slice(0, -1));
  };

  return (
    <div className="bg-white p-4 rounded-xl border border-gray-200 shadow-sm transition-all">
      <h3 className="font-bold text-gray-700 text-sm mb-3 flex items-center gap-2">
        🎨 스마트 도안 에디터
        {step === 'editing' && <span className="text-[10px] bg-red-100 text-red-600 px-2 rounded-full animate-pulse">REC</span>}
      </h3>
      
      {/* 1. 파일 선택 모드 */}
      {step === 'upload' && (
          <div className="flex flex-col gap-3">
            <input 
                type="file" 
                accept="image/*" 
                onChange={handleFileChange} 
                className="block w-full text-xs text-slate-500 file:mr-2 file:py-1 file:px-3 file:rounded-full file:border-0 file:text-xs file:font-bold file:bg-purple-50 file:text-purple-700 hover:file:bg-purple-100 cursor-pointer" 
            />
            
            {preview && (
                 <div className="w-full h-32 bg-gray-100 rounded-lg overflow-hidden border border-gray-200">
                    <img src={preview} className="w-full h-full object-contain opacity-50" />
                 </div>
            )}

            <button 
                onClick={startEditing} 
                disabled={!selectedFile || isProcessing}
                className={`w-full py-2 rounded-lg font-bold text-sm shadow-sm transition-all
                    ${!selectedFile || isProcessing ? 'bg-gray-300 text-gray-500' : 'bg-purple-600 text-white hover:bg-purple-500'}`}
            >
                {isProcessing ? "분석 중..." : "이미지 분석 및 편집 시작"}
            </button>
          </div>
      )}

      {/* 2. 편집 모드 (Canvas) */}
      {step === 'editing' && (
          <div className="flex flex-col gap-2 items-center">
             <div className="relative border border-gray-300 shadow-inner bg-gray-50 cursor-crosshair touch-none rounded-lg overflow-hidden">
                 <canvas 
                    ref={canvasRef}
                    onMouseDown={onMouseDown}
                    onMouseMove={onMouseMove}
                    onMouseUp={onMouseUp}
                    onMouseLeave={onMouseUp}
                    className="block"
                 />
                 {/* 범례 */}
                 <div className="absolute top-2 left-2 flex flex-col gap-1 text-[10px] bg-white/90 p-2 rounded shadow backdrop-blur-sm">
                    <span className="text-green-600 font-bold flex items-center gap-1"><span className="w-2 h-2 rounded-full bg-green-500"></span> 자동 경로</span>
                    <span className="text-red-600 font-bold flex items-center gap-1"><span className="w-2 h-2 rounded-full bg-red-500"></span> 수동 추가</span>
                 </div>
             </div>
             
             <div className="flex gap-2 w-full mt-2">
                 <button onClick={undoManual} className="flex-1 bg-gray-100 text-gray-600 py-2 rounded-lg text-xs font-bold hover:bg-gray-200 border">
                    ↩️ 실행 취소
                 </button>
                 <button onClick={() => {setManualPaths([]); setAutoPaths([]);}} className="flex-1 bg-red-50 text-red-500 py-2 rounded-lg text-xs font-bold hover:bg-red-100 border border-red-100">
                    🗑️ 모두 지우기
                 </button>
             </div>

             <button 
                onClick={handleSave}
                disabled={isProcessing}
                className="w-full bg-blue-600 text-white py-3 rounded-lg font-bold text-sm mt-2 hover:bg-blue-500 shadow-md transition-transform active:scale-[0.98]"
             >
                {isProcessing ? "저장 중..." : "✅ 편집 완료 및 저장"}
             </button>
          </div>
      )}

      {/* 3. 완료 상태 */}
      {step === 'done' && (
          <div className="text-center py-6 bg-green-50 rounded-lg border border-green-100">
              <div className="text-4xl mb-2">🎉</div>
              <p className="font-bold text-green-800 text-sm">도안 설정이 완료되었습니다!</p>
              <p className="text-xs text-green-600 mt-1">이제 '주문서 전송'을 눌러 로봇을 작동시키세요.</p>
              <button onClick={() => setStep('upload')} className="mt-4 text-xs text-gray-400 underline hover:text-gray-600">
                  새 도안 만들기
              </button>
          </div>
      )}
    </div>
  );
}