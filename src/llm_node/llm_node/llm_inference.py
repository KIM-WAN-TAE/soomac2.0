# -*- coding: utf-8 -*-
"""
tool_chat_engine.py
- 기존 CLI 로직을 클래스화하여, ROS2 노드에서 import 사용 가능하도록 구성
- 의도/모드/툴 추론 알고리즘과 부정/대조 처리, RAG/LLM 보강 로직은 그대로 유지
"""
import os, re, json, time, unicodedata
from textwrap import dedent
from typing import Optional, Dict, Any, Tuple, List

import numpy as np
from pydantic import Field, model_validator

# LLM/RAG 관련
from llama_cpp import Llama
from langchain.docstore.document import Document
from langchain_community.vectorstores import FAISS
from langchain.chains import RetrievalQA
from langchain_community.llms import LlamaCpp
from langchain.prompts import PromptTemplate
from langchain_core.retrievers import BaseRetriever

class ToolChatEngine:
    """
    - 모든 추론 로직을 보존한 클래스 버전
    - ROS2 노드에서 이 클래스를 초기화해 사용 (model/RAG 한 번만 로드)
    """
    # ── 상수/화이트리스트 ───────────────────────────────────
    CANDIDATES = {"START", "DELIVER", "RETURN", "CLEAN", "ASSEMBLE", "FINISH"}
    _MODE_RE = re.compile(r"\b(START|DELIVER|RETURN|CLEAN|ASSEMBLE|FINISH)\b", re.IGNORECASE)

    QUESTION_CUES = {
        "뭐지", "뭐였지", "어떤 도구", "무슨 도구",
        "알려줘", "추천", "어떻게 하지", "뭘 써",
        "뭘로", "뭐로", "무엇으로"
    }
    COMMAND_CUES  = {"줘", "줄래", "주세요", "가져다", "가져와", "건네", "필요해"}

    MODE_ALIASES = {
        "START":   ["시작", "안녕", "hello", "하이", "준비"],
        "DELIVER": ["가져다줘", "가져다 줘", "가져와줘", "가져와", "전달", "넘겨줘", "핸드오프", "줄래", "주세요"],
        "RETURN":  ["원위치", "제자리", "반납", "되돌려놔", "돌려놔", "돌려줘"],
        "CLEAN":   ["청소 시작", "청소", "치워", "정돈", "닦"],
        "ASSEMBLE":["결합", "체결 시작", "체결", "조립", "조여", "풀어", "고정", "분해"],
        "FINISH":  ["마무리", "종료", "수고했어", "그만", "끝내"]
    }

    ALLOWED_TOOLS = {
        "육각 렌치": ["육각렌치", "육각 렌치", "헥사 렌치", "헥스키", "육각키", "hex key", "hex", "알렌키"],
        "디지털 버니어 캘리퍼스": ["디지털 버니어 캘리퍼스", "버니어", "버니어 캘리퍼스", "버니어캘리퍼스", "캘리퍼스", "디지털 캘리퍼스"],
        "롱노우즈 플라이어": ["롱노우즈 플라이어", "롱노즈 플라이어", "롱노우즈", "롱노즈", "롱 플라이어", "롱플라이어"],
        "정밀 니퍼": ["정밀 니퍼", "니퍼", "precision nipper"],
    }
    MODE_TOOL_REQUIRED = {"DELIVER", "RETURN"}

    OPERATION_ALIASES = {
        "measure":  ["재", "측정", "길이", "크기", "직경", "내경", "외경", "깊이", "단차"],
        "grip":     ["잡", "집", "고정"],
        "retrieve": ["꺼내", "꺼내줘", "뽑", "집어내"],
        "cut":      ["자르", "잘라", "자를", "잘라줘", "컷", "끊", "끊어", "커팅", "절단"],
        "fasten":   ["조이", "잠그", "체결", "고정"],
        "unfasten": ["풀어", "해체", "분리"],
    }

    OBJECT_ALIASES = {
        "볼트": ["볼트", "나사", "육각볼트", "알렌볼트"],
        "너트": ["너트"],
        "전선": ["전선", "케이블", "리드선", "케이블 타이", "타이"],
        "PCB": ["pcb", "회로기판", "보드"],
        "구멍(내경)": ["구멍", "내경", "홀"],
        "큐브": ["큐브", "정육면체", "미니 큐브"],
    }

    _NEG_TOKENS_RE = r"(하지\s*말|하지마|말고|말라|금지|빼고|제외|안\s*해|하지\s*않|못\s*해)"
    MODE_KEYWORDS = {
        "DELIVER":  ["가져와", "가져다", "가져와줘", "가져다줘", "줘", "주라", "주세요", "건네", "전달", "넘겨"],
        "RETURN":   MODE_ALIASES["RETURN"],
        "CLEAN":    ["청소", "치우", "정돈", "닦"],
        "ASSEMBLE": ["체결", "조립", "조여", "풀", "고정", "분해"],
        "START":    ["시작", "준비"],
        "FINISH":   ["마무리", "종료", "끝내", "그만"],
    }

    # ── 간이 색상 (옵션) ────────────────────────────────────
    class C:
        GREEN, YELLOW, RED, CYAN, END = '\033[92m', '\033[93m', '\033[91m', '\033[96m', '\033[0m'

    # ── 간이 TF-IDF Retriever ───────────────────────────────
    class SimpleTfidfRetriever(BaseRetriever):
        docs: list[Document]
        k: int = 3
        tfidf_mat: np.ndarray = Field(default=None, repr=False)
        idf: np.ndarray = Field(default=None, repr=False)
        vocab: dict = Field(default=None, repr=False)

        class Config:
            arbitrary_types_allowed = True

        @model_validator(mode='after')
        def build_tfidf(self) -> 'ToolChatEngine.SimpleTfidfRetriever':
            if self.tfidf_mat is not None: return self
            tokens_per_doc = [ToolChatEngine._tokenize(d.page_content) for d in self.docs]
            vocab = {}
            for toks in tokens_per_doc:
                for t in toks:
                    if t not in vocab: vocab[t] = len(vocab)
            self.vocab = vocab
            V, N = len(vocab), len(self.docs)
            df = np.zeros(V, dtype=np.float32)
            for toks in tokens_per_doc:
                for t in set(toks):
                    if t in vocab: df[vocab[t]] += 1.0
            self.idf = np.log((N + 1) / (df + 1)) + 1.0
            self.tfidf_mat = np.zeros((N, V), dtype=np.float32)
            for i, toks in enumerate(tokens_per_doc):
                if not toks: continue
                tf = {t: toks.count(t) for t in set(toks)}
                vec = np.zeros(V, dtype=np.float32)
                for t, c in tf.items():
                    if t in self.vocab:
                        j = self.vocab[t]
                        vec[j] = (c / len(toks)) * self.idf[j]
                norm = np.linalg.norm(vec)
                self.tfidf_mat[i, :] = vec / (norm + 1e-9)
            return self

        def _get_relevant_documents(self, query: str) -> list[Document]:
            q_toks = ToolChatEngine._tokenize(query)
            if not q_toks or not self.vocab: return self.docs[:self.k]
            V = len(self.vocab)
            q_vec = np.zeros(V, dtype=np.float32)
            tf = {t: q_toks.count(t) for t in set(q_toks) if t in self.vocab}
            if not tf: return self.docs[:self.k]
            for t, c in tf.items():
                j = self.vocab[t]
                q_vec[j] = (c / len(q_toks)) * self.idf[j]
            q_norm = np.linalg.norm(q_vec)
            if q_norm < 1e-9: return self.docs[:self.k]
            q_vec = q_vec / q_norm
            sims = self.tfidf_mat @ q_vec
            top_idx = np.argsort(-sims)[:self.k]
            return [self.docs[i] for i in top_idx]

    # ── 유틸 ────────────────────────────────────────────────
    @staticmethod
    def _normalize_kr(s: str) -> str:
        s = unicodedata.normalize('NFKC', s or "")
        s = s.lower()
        s = re.sub(r"\s+", " ", s).strip()
        return s

    _WORD_RE = re.compile(r"[A-Za-z0-9]+|[가-힣]+")
    @staticmethod
    def _tokenize(text: str):
        return ToolChatEngine._WORD_RE.findall((text or "").lower())

    @staticmethod
    def _contains_kw(t: str, kws: list[str]) -> bool:
        t = ToolChatEngine._normalize_kr(t); return any(k in t for k in kws)

    @staticmethod
    def _is_negated_action(t: str, kws: list[str]) -> bool:
        t = ToolChatEngine._normalize_kr(t)
        neg = ToolChatEngine._NEG_TOKENS_RE
        for kw in kws:
            if re.search(rf"{re.escape(kw)}\s*(은|는|이|가|을|를)?\s*{neg}", t):
                return True
            if re.search(rf"(안|못)\s*{re.escape(kw)}", t):
                return True
            if re.search(rf"{re.escape(kw)}\s*(안|못)\s*(해|하|하기|하세요|해줘)", t):
                return True
        return False

    @staticmethod
    def _is_imperative(t: str, kws: list[str]) -> bool:
        t = ToolChatEngine._normalize_kr(t)
        imp = r"(해|해줘|해라|해줄래|하시|하세요|하자|시작|진행|줘|주세요|가져와|가져다줘|반납|원위치)"
        for kw in kws:
            if re.search(rf"{re.escape(kw)}.{{0,8}}{imp}", t):
                return True
        return False

    @staticmethod
    def _contrast_preferred_mode(t: str) -> Optional[str]:
        t = ToolChatEngine._normalize_kr(t)
        def _alt(kws: list[str]) -> str:
            return "|".join(map(re.escape, kws))
        for A, akw in ToolChatEngine.MODE_KEYWORDS.items():
            for B, bkw in ToolChatEngine.MODE_KEYWORDS.items():
                if A == B:
                    continue
                pat = rf"(({_alt(akw)})).{{0,16}}{ToolChatEngine._NEG_TOKENS_RE}.{{0,32}}(({_alt(bkw)}))"
                if re.search(pat, t):
                    return B
        return None

    # ── 초기화 ───────────────────────────────────────────────
    def __init__(self,
                 model_path: str = "/home/temp_id/LLM/gemma3-q4_k_m_budda.gguf",
                 rag_data_path: str = "/home/temp_id/SooMac/main_tool.json",
                 use_gpu_llama: bool = False,
                 verbose: bool = True):
        self.verbose = verbose
        self.model_path = model_path
        self.rag_data_path = rag_data_path
        self.n_gpu_layers = -1 if use_gpu_llama else 0

        # 세션 상태
        self.session_state: Dict[str, Any] = {
            "last_tool": None,
            "last_mode": None,
            "last_object": None,
        }

        if self.verbose:
            print(f"[*] GGUF 모델 로드: {self.model_path}", flush=True)
        self.llm_model = Llama(model_path=self.model_path, n_ctx=4096, n_batch=512,
                               n_gpu_layers=self.n_gpu_layers, seed=0, verbose=False)
        self.llm_chain = LlamaCpp(model_path=self.model_path, n_ctx=4096,
                                  n_gpu_layers=self.n_gpu_layers, temperature=0.1, verbose=False)

        self.rag_chain = self._setup_rag_pipeline()

    # ── RAG 파이프라인 ──────────────────────────────────────
    def _setup_rag_pipeline(self):
        if self.verbose:
            print("[*] RAG 데이터베이스 설정...", flush=True)
        if not os.path.isfile(self.rag_data_path):
            print(f"{self.C.RED}[X] RAG 데이터 파일이 없음: {self.rag_data_path}{self.C.END}")
            return None

        docs: List[Document] = []
        try:
            with open(self.rag_data_path, 'r', encoding='utf-8') as f:
                full_data = json.load(f)
        except Exception as e:
            print(f"{self.C.RED}[X] JSON 로드 실패: {e}{self.C.END}")
            return None

        for tool in full_data.get('tools', []):
            tool_name = tool.get('tool_name', 'Unknown')
            metadata = {"tool": tool_name}
            if tool.get('function_mapping'):
                docs.append(Document(page_content=tool['function_mapping'], metadata=metadata))
            if tool.get('overview', {}).get('definition'):
                docs.append(Document(page_content=tool['overview']['definition'], metadata=metadata))
            for qa in tool.get('q_and_a', []):
                docs.append(Document(page_content=f"질문: {qa.get('question','')}\n답변: {qa.get('answer','')}", metadata=metadata))

        if not docs:
            print(f"{self.C.RED}JSON에서 문서를 만들지 못함. 스키마 확인.{self.C.END}")
            return None

        try:
            from langchain_huggingface import HuggingFaceEmbeddings
            embeddings = HuggingFaceEmbeddings(
                model_name='jhgan/ko-sbert-nli',
                model_kwargs={'device': 'cpu'},
                encode_kwargs={'normalize_embeddings': True}
            )
            vectorstore = FAISS.from_documents(docs, embeddings)
            retriever = vectorstore.as_retriever(search_kwargs={'k': 3})
            if self.verbose:
                print(f"{self.C.CYAN}[*] Retriever: HF Embeddings + FAISS (성공){self.C.END}")
        except Exception as e:
            if self.verbose:
                print(f"{self.C.YELLOW}[!] 임베딩 로드 실패 → TF-IDF 폴백: {e}{self.C.END}")
            retriever = self.SimpleTfidfRetriever(docs=docs, k=3)

        prompt_template = (
            "당신은 사용자의 질문에 가장 적합한 공구를 찾아주는 전문가입니다.\n"
            "주어진 정보(Context)를 바탕으로 질문에서 묘사하는 기능을 수행하는 공구의 이름을 명확하게 알려주세요.\n"
            "만약 적절한 공구를 찾았다면, \"그 작업을 위한 공구는 [공구 이름]입니다.\" 형식으로만 답하세요\n\n"
            "Context: {context}\nQuestion: {question}\nAnswer:"
        )
        rag_prompt = PromptTemplate(template=prompt_template, input_variables=["context", "question"])
        qa_chain = RetrievalQA.from_chain_type(llm=self.llm_chain, chain_type="stuff",
                                               retriever=retriever,
                                               chain_type_kwargs={"prompt": rag_prompt})
        if self.verbose:
            print("[*] RAG 설정 완료.", flush=True)
        return qa_chain

    # ── 분류/정규화 ─────────────────────────────────────────
    def classify_intent(self, user_text: str) -> str:
        txt = self._normalize_kr(user_text)
        if any(cue in txt for cue in self.COMMAND_CUES):
            return "명령"
        for aliases in self.MODE_ALIASES.values():
            if any(a in txt for a in aliases):
                return "명령"
        if any(q in txt for q in self.QUESTION_CUES):
            return "질문"

        prompt = (
            "### System\n역할: 사용자의 발화 의도를 '명령', '질문', '인사말', '마무리', '기타' 중 하나로 분류하세요.\n출력 형식: {Intent: <분류>}\n\n"
            "### User\n렌치로 너트 좀 올려줘\n### Assistant\n{Intent: 명령}\n\n"
            "### User\n스트리퍼가 뭐야?\n### Assistant\n{Intent: 질문}\n\n"
            "### User\n안녕! 시작해볼까\n### Assistant\n{Intent: 인사말}\n\n"
            "### User\n여기까지 하자\n### Assistant\n{Intent: 마무리}\n\n"
            "### User\n고마워\n### Assistant\n{Intent: 기타}\n\n"
            f"### User\n{user_text}\n### Assistant\n"
        )
        try:
            raw = self.llm_model.create_completion(prompt=prompt, max_tokens=16, temperature=0.0, stop=["}"])['choices'][0]['text']
            for k in ("명령","질문","인사말","마무리"):
                if k in raw: return k
        except Exception:
            pass
        return "기타"

    def canonicalize_mode(self, user_text: str) -> Optional[str]:
        txt = self._normalize_kr(user_text)
        for mode, aliases in self.MODE_ALIASES.items():
            if any(a in txt for a in aliases):
                return mode
        return None

    def canonicalize_tool(self, user_text: str) -> Optional[str]:
        txt = self._normalize_kr(user_text)
        for canon, alias_list in self.ALLOWED_TOOLS.items():
            for alias in alias_list:
                if self._normalize_kr(alias) in txt:
                    return canon
        return None

    def detect_operation(self, user_text: str) -> Optional[str]:
        txt = self._normalize_kr(user_text)
        for op, cues in self.OPERATION_ALIASES.items():
            if any(c in txt for c in cues):
                return op
        return None

    def detect_object(self, user_text: str) -> Optional[str]:
        txt = self._normalize_kr(user_text)
        for obj, cues in self.OBJECT_ALIASES.items():
            if any(self._normalize_kr(c) in txt for c in cues):
                return obj
        return None

    def is_question_only(self, user_text: str) -> bool:
        txt = self._normalize_kr(user_text)
        if any(c in txt for c in self.COMMAND_CUES):
            return False
        return any(q in txt for q in self.QUESTION_CUES)

    def decide_mode(self, user_text: str, base_mode: Optional[str]) -> Optional[str]:
        t = self._normalize_kr(user_text)
        pref = self._contrast_preferred_mode(t)
        if pref:
            return pref

        present = {m: self._contains_kw(t, self.MODE_KEYWORDS[m]) for m in self.MODE_KEYWORDS}
        neg     = {m: self._is_negated_action(t, self.MODE_KEYWORDS[m]) for m in self.MODE_KEYWORDS}
        imp     = {m: self._is_imperative(t, self.MODE_KEYWORDS[m]) for m in self.MODE_KEYWORDS}

        candidates = [m for m in self.MODE_KEYWORDS if present[m] and not neg[m]]
        if base_mode and not neg.get(base_mode, False) and base_mode not in candidates:
            candidates.append(base_mode)

        imp_candidates = [m for m in candidates if imp[m]]
        if imp_candidates:
            candidates = imp_candidates

        PRIORITY = ["RETURN", "CLEAN", "ASSEMBLE", "START", "FINISH", "DELIVER"]
        for m in PRIORITY:
            if m in candidates:
                return m

        if present["DELIVER"] and not neg["DELIVER"]:
            return "DELIVER"
        return base_mode

    # ── LLM 보조 ────────────────────────────────────────────
    def _extract_mode_token(self, text: str) -> Optional[str]:
        if not text: return None
        m = self._MODE_RE.search(text.strip())
        return m.group(1).upper() if m else None

    def _extract_tool_from_lines(self, text: str) -> Optional[str]:
        if not text: return None
        t = text.strip()
        for line in t.splitlines():
            line = line.strip()
            cand = line.split(":",1)[1].strip() if ":" in line else line
            for canon, aliases in self.ALLOWED_TOOLS.items():
                if canon == cand:
                    return canon
                for alias in aliases:
                    if self._normalize_kr(alias) == self._normalize_kr(cand):
                        return canon
        for canon, aliases in self.ALLOWED_TOOLS.items():
            if canon in t: return canon
            for alias in aliases:
                if self._normalize_kr(alias) in self._normalize_kr(t):
                    return canon
        return None

    def llm_guess_mode_and_tool(self, user_text: str, operation: Optional[str], obj: Optional[str]) -> Tuple[Optional[str], Optional[str], str]:
        choices = " | ".join(sorted(self.ALLOWED_TOOLS.keys())) + " | NONE"
        ex = (
            "예시1) 문장: 선 정리 좀 하게, 자를 것 좀 가져와\n"
            "Mode: DELIVER\n"
            "Tool: 정밀 니퍼\n\n"
            "예시2) 문장: 그거 청소하지 말고 니퍼만 가져와\n"
            "Mode: DELIVER\n"
            "Tool: 정밀 니퍼\n\n"
            "예시3) 문장: 케이블 타이 잘라\n"
            "Mode: ASSEMBLE\n"
            "Tool: 정밀 니퍼\n\n"
            "예시4) 문장: 니퍼 원위치해\n"
            "Mode: RETURN\n"
            "Tool: 정밀 니퍼\n\n"
        )
        ctx_bits = []
        if operation: ctx_bits.append(f"operation={operation}")
        if obj:       ctx_bits.append(f"object={obj}")
        ctx = ("문맥: " + ", ".join(ctx_bits) + "\n") if ctx_bits else ""
        prompt = (
            "가능한 Mode: START | DELIVER | RETURN | CLEAN | ASSEMBLE | FINISH\n"
            f"가능한 Tool: {choices}\n"
            "규칙: 아래 형식으로만 두 줄 출력. 설명/따옴표/추가 텍스트 금지.\n"
            "Mode: <위 여섯 중 하나>\n"
            "Tool: <정식명 또는 NONE>\n\n"
            + ex + ctx +
            f"문장: {user_text}\n"
            "Mode: "
        )
        raw = self.llm_model.create_completion(
            prompt=prompt, max_tokens=16, temperature=0.0, stop=["\n\n"]
        )['choices'][0]['text']
        text_out = "Mode: " + (raw or "").strip()
        mode = self._extract_mode_token(text_out)
        tool = self._extract_tool_from_lines(text_out)
        if tool and tool not in self.ALLOWED_TOOLS:
            tool = None
        return mode, tool, text_out

    # ── 도구 매핑/확인 ──────────────────────────────────────
    def map_tool(self, operation: Optional[str], obj: Optional[str], user_text: str) -> Optional[str]:
        txt = self._normalize_kr(user_text)
        if operation == "measure":
            return "디지털 버니어 캘리퍼스"
        if operation in {"grip", "retrieve"}:
            return "롱노우즈 플라이어"
        if operation == "cut":
            return "정밀 니퍼"
        if operation in {"fasten", "unfasten"}:
            if any(k in txt for k in ["육각", "알렌", "hex"]):
                return "육각 렌치"
            if obj in {"볼트"}:
                return "육각 렌치"
        explicit = self.canonicalize_tool(user_text)
        if explicit:
            return explicit
        return None

    def need_confirmation(self, tool: Optional[str], operation: Optional[str], obj: Optional[str], user_text: str) -> bool:
        txt = self._normalize_kr(user_text)
        if operation in {"fasten", "unfasten"} and tool == "육각 렌치":
            if not any(k in txt for k in ["육각", "알렌", "hex"]):
                return True
        return False

    # ── 외부 API ────────────────────────────────────────────
    def parse_and_infer(self, user_text: str) -> Dict[str, Any]:
        intent    = self.classify_intent(user_text)
        base_mode = self.canonicalize_mode(user_text)
        operation = self.detect_operation(user_text)
        obj       = self.detect_object(user_text)
        tool      = self.canonicalize_tool(user_text) or self.map_tool(operation, obj, user_text)

        if self.verbose:
            print(f"{self.C.CYAN}[디버그] intent={intent}, base_mode={base_mode}, operation={operation}, object={obj}, tool_suggested={tool}{self.C.END}", flush=True)

        mode = self.decide_mode(user_text, base_mode=base_mode)

        # 모드 미결이면 LLM 보조
        if not mode and intent == "명령":
            try:
                mode_llm, tool_llm, raw = self.llm_guess_mode_and_tool(user_text, operation, obj)
                if mode_llm in self.CANDIDATES:
                    mode = self.decide_mode(user_text, base_mode=mode_llm)
                if (tool_llm in self.ALLOWED_TOOLS) and (not tool):
                    tool = tool_llm
                if self.verbose:
                    print(f"{self.C.CYAN}[디버그] LLM 유추: {repr(raw)} → mode={mode}, tool={tool}{self.C.END}", flush=True)
            except Exception as e:
                if self.verbose:
                    print(f"{self.C.YELLOW}[경고] LLM 유추 실패: {e}{self.C.END}", flush=True)

        if self.verbose:
            print(f"{self.C.CYAN}[디버그] 최종 모드 결정: {mode}{self.C.END}", flush=True)

        question_only = self.is_question_only(user_text)
        ask_back = self.need_confirmation(tool, operation, obj, user_text)

        should_call = False
        function_name = ""
        function_args: Dict[str, Any] = {}

        if intent == "명령" and not question_only and mode:
            if mode in {"START", "FINISH", "CLEAN", "ASSEMBLE"}:
                should_call = True
                function_name = mode.lower()
                function_args = {}
            elif mode == "DELIVER":
                if not tool:
                    ask_back = True
                else:
                    should_call = not ask_back
                    function_name = "deliver_tool"
                    function_args = {"tool": tool, "target": "user", "note": operation or ""}
            elif mode == "RETURN":
                if not tool:
                    ask_back = True
                else:
                    should_call = True
                    function_name = "return_tool"
                    function_args = {"tool": tool}

        if question_only:
            should_call = False
            function_name = ""
            function_args = {}

        if should_call and function_name == "deliver_tool" and tool:
            self.session_state["last_tool"]  = tool
            self.session_state["last_mode"]  = "DELIVER"
            self.session_state["last_object"]= obj

        return {
            "intent": intent,
            "mode": mode or "NONE",
            "operation": operation or "NONE",
            "object": obj or "NONE",
            "tool_final": tool or "NONE",
            "ask_back": bool(ask_back),
            "question_only": question_only,
            "should_call_function": should_call,
            "function_name": function_name,
            "function_args": function_args,
        }

    def smalltalk_reply(self, user_text: str) -> str:
        txt = self._normalize_kr(user_text)
        if any(k in txt for k in ["고마워", "감사", "thanks", "thx"]):
            return "도움이 되어 기뻐요. 다른 것도 도와드릴게요."
        if any(k in txt for k in ["안녕", "hello", "하이"]):
            return "안녕하세요! 준비됐어요. 무엇을 도와드릴까요?"
        prompt = (
            "### 시스템\n"
            "너는 짧고 공손한 한국어 일상 대화 봇이야. 한두 문장, 80자 이내로 답해.\n"
            "지시/명령/코드/링크/이모지/과장 금지.\n"
            f"사용자: {user_text}\n어시스턴트:"
        )
        try:
            out = self.llm_model.create_completion(prompt=prompt, max_tokens=60, temperature=0.6, stop=["\n"])['choices'][0]['text'].strip()
            if len(out) > 80:
                out = out[:80]
            return out or "그렇군요. 더 도와드릴 게 있을까요?"
        except Exception:
            return "그렇군요. 더 도와드릴 게 있을까요?"

    # RAG/LLM로 툴 보강
    def _extract_tool_from_text(self, text: str) -> Optional[str]:
        tnorm = self._normalize_kr(text)
        for canon, aliases in self.ALLOWED_TOOLS.items():
            for alias in aliases:
                if self._normalize_kr(alias) in tnorm:
                    return canon
        return None

    def rag_pick_tool(self, user_text: str, operation: Optional[str], obj: Optional[str]):
        if self.rag_chain is None:
            return None, ""
        q = user_text or ""
        hints = []
        if obj and obj != "NONE": hints.append(obj)
        if operation and operation != "NONE": hints.append(operation)
        if hints: q = f"{' '.join(hints)} 작업에 적합한 공구는?"
        try:
            res = self.rag_chain.invoke(q)
            ans = (res.get('result', '') or '').strip()
            tool = self._extract_tool_from_text(ans)
            return tool, ans
        except Exception:
            return None, ""

    def llm_pick_tool(self, operation: Optional[str], obj: Optional[str], user_text: str) -> Optional[str]:
        try:
            choices = " | ".join(sorted(self.ALLOWED_TOOLS.keys()))
            desc = []
            if obj and obj != "NONE":       desc.append(f"대상={obj}")
            if operation and operation != "NONE": desc.append(f"작업={operation}")
            if user_text:                   desc.append(f"문장={user_text}")
            ctx = " | ".join(desc) or user_text
            prompt = (
                "역할: 아래 작업에 가장 적합한 공구를 선택지에서 하나만 골라 정식명으로 답하라. "
                f"선택지: {choices} "
                "출력 형식: {tool: <정식명 또는 NONE>} "
                f"작업 설명: {ctx}응답: "
            )
            raw = self.llm_model.create_completion(prompt=prompt, max_tokens=16, temperature=0.0, stop=["}"])['choices'][0]['text']
            for canon in self.ALLOWED_TOOLS.keys():
                if canon in raw:
                    return canon
            if "NONE" in raw.upper():
                return None
        except Exception:
            pass
        return None

    # ── 실행 계획(이전 simulate_vla_action)을 "계획 반환"으로 변경 ──
    def plan_action(self, parsed: Dict[str, Any], raw_text: str = "") -> Dict[str, Any]:
        """
        simulate_vla_action과 동일 로직으로 '무엇을 실행할지'를 결정해 딕셔너리로 반환.
        실제 실행/출력은 하지 않음(ROS 노드가 퍼블리시/실행 담당).
        """
        mode = parsed.get("mode")
        tool = parsed.get("tool_final")
        fn   = parsed.get("function_name")
        args = parsed.get("function_args")
        ask  = parsed.get("ask_back")
        operation = parsed.get("operation")
        obj = parsed.get("object")

        # 모드/툴 보강 (LLM)
        if (not mode) or (mode == "NONE"):
            try:
                mode2, tool2, raw = self.llm_guess_mode_and_tool(raw_text,
                                                                 operation if operation != "NONE" else None,
                                                                 obj if obj != "NONE" else None)
                if mode2 in self.CANDIDATES:
                    mode = mode2; parsed["mode"] = mode2
                if (not tool or tool == "NONE") and (tool2 in self.ALLOWED_TOOLS):
                    tool = tool2; parsed["tool_final"] = tool2
                if self.verbose:
                    print(f"{self.C.CYAN}[디버그] (보강) LLM 유추: {repr(raw)} → mode={mode}, tool={tool}{self.C.END}", flush=True)
            except Exception as e:
                if self.verbose:
                    print(f"{self.C.YELLOW}[경고] 실행 직전 LLM 보강 실패: {e}{self.C.END}", flush=True)

        # DELIVER/RETURN에서 툴 미지정 → RAG→LLM 순 보강
        if (mode in {"DELIVER", "RETURN"}) and (not tool or tool == "NONE"):
            picked, _ = self.rag_pick_tool(raw_text,
                                           operation if operation != "NONE" else None,
                                           obj if obj != "NONE" else None)
            if picked:
                if self.verbose: print(f"{self.C.CYAN}[추천] RAG 후보: {picked}{self.C.END}")
                tool = picked
            else:
                picked = self.llm_pick_tool(operation if operation != "NONE" else None,
                                            obj if obj != "NONE" else None, raw_text)
                if picked:
                    if self.verbose: print(f"{self.C.CYAN}[추천] LLM 후보: {picked}{self.C.END}")
                    tool = picked

            if tool and tool != "NONE":
                ask = self.need_confirmation(tool,
                                             operation if operation != "NONE" else None,
                                             obj if obj != "NONE" else None,
                                             raw_text)
                if mode == "DELIVER":
                    fn = "deliver_tool"; args = {"tool": tool, "target": "user", "note": (operation if operation != "NONE" else "")}
                elif mode == "RETURN":
                    fn = "return_tool";  args = {"tool": tool}

        # 최종 확인 필요 여부
        if (mode in {"DELIVER", "RETURN"}) and tool and tool != "NONE":
            ask = self.need_confirmation(tool,
                                         operation if operation != "NONE" else None,
                                         obj if obj != "NONE" else None,
                                         raw_text)

        # should_call 갱신
        should_call = parsed.get("should_call_function")
        if not should_call and mode:
            if mode in {"START", "FINISH", "CLEAN", "ASSEMBLE"}:
                should_call = True; fn = mode.lower(); args = {}
            elif mode == "DELIVER" and tool and tool != "NONE" and not ask:
                should_call = True; fn = "deliver_tool"; args = {"tool": tool, "target": "user", "note": (operation if operation != "NONE" else '')}
            elif mode == "RETURN" and tool and tool != "NONE" and not ask:
                should_call = True; fn = "return_tool"; args = {"tool": tool}

        # 세션 상태 업데이트
        if should_call and fn == "deliver_tool" and tool:
            self.session_state["last_tool"] = tool
            self.session_state["last_mode"] = "DELIVER"
            self.session_state["last_object"] = obj if obj != "NONE" else None

        # 사용자 확인이 필요한 경우 메시지 제안
        confirm_text = ""
        if ask:
            if mode == "DELIVER" and (not tool or tool == "NONE"):
                confirm_text = "어떤 공구가 필요하신가요? (예: 육각 렌치/버니어/롱노우즈/니퍼)"
            elif mode == "RETURN" and (not tool or tool == "NONE"):
                lt = self.session_state.get('last_tool')
                confirm_text = f"반납할 공구가 무엇인가요? 최근 전달 공구: {lt}"
            elif mode in {"ASSEMBLE", "CLEAN"}:
                confirm_text = ""  # 로깅만

        return {
            "mode": mode,
            "tool_final": tool,
            "should_call_function": bool(should_call),
            "function_name": fn or "",
            "function_args": args or {},
            "ask_back": bool(ask),
            "confirm_text": confirm_text,
        }
