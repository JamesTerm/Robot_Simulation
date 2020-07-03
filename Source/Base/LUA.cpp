#if 1
#include "LUA.h"

#ifdef _Win32
// No C library depreciation warnings
#pragma warning ( disable : 4995 )
#pragma warning ( disable : 4996 )
#pragma warning ( disable : 4477 )

#define _CRT_SECURE_NO_WARNINGS
#endif

//-------------------------------------------------------------lfunc.c------------------------------------------------------
//from lgc.h
LUAI_FUNC size_t luaC_separateudata (lua_State *L, int all);
LUAI_FUNC void luaC_callGCTM (lua_State *L);
LUAI_FUNC void luaC_freeall (lua_State *L);
LUAI_FUNC void luaC_step (lua_State *L);
LUAI_FUNC void luaC_fullgc (lua_State *L);
LUAI_FUNC void luaC_link (lua_State *L, GCObject *o, lu_byte tt);
LUAI_FUNC void luaC_linkupval (lua_State *L, UpVal *uv);
LUAI_FUNC void luaC_barrierf (lua_State *L, GCObject *o, GCObject *v);
LUAI_FUNC void luaC_barrierback (lua_State *L, Table *t);

Closure *luaF_newCclosure (lua_State *L, int nelems, Table *e) {
	Closure *c = cast(Closure *, luaM_malloc(L, sizeCclosure(nelems)));
	luaC_link(L, obj2gco(c), LUA_TFUNCTION);
	c->c.isC = 1;
	c->c.env = e;
	c->c.nupvalues = cast_byte(nelems);
	return c;
}


Closure *luaF_newLclosure (lua_State *L, int nelems, Table *e) {
	Closure *c = cast(Closure *, luaM_malloc(L, sizeLclosure(nelems)));
	luaC_link(L, obj2gco(c), LUA_TFUNCTION);
	c->l.isC = 0;
	c->l.env = e;
	c->l.nupvalues = cast_byte(nelems);
	while (nelems--) c->l.upvals[nelems] = NULL;
	return c;
}


UpVal *luaF_newupval (lua_State *L) {
	UpVal *uv = luaM_new(L, UpVal);
	luaC_link(L, obj2gco(uv), LUA_TUPVAL);
	uv->v = &uv->u.value;
	setnilvalue(uv->v);
	return uv;
}


UpVal *luaF_findupval (lua_State *L, StkId level) {
	global_State *g = G(L);
	GCObject **pp = &L->openupval;
	UpVal *p;
	UpVal *uv;
	while (*pp != NULL && (p = ngcotouv(*pp))->v >= level) {
		lua_assert(p->v != &p->u.value);
		if (p->v == level) {  /* found a corresponding upvalue? */
			if (isdead(g, obj2gco(p)))  /* is it dead? */
				changewhite(obj2gco(p));  /* ressurect it */
			return p;
		}
		pp = &p->next;
	}
	uv = luaM_new(L, UpVal);  /* not found: create a new one */
	uv->tt = LUA_TUPVAL;
	uv->marked = luaC_white(g);
	uv->v = level;  /* current value lives in the stack */
	uv->next = *pp;  /* chain it in the proper position */
	*pp = obj2gco(uv);
	uv->u.l.prev = &g->uvhead;  /* double link it in `uvhead' list */
	uv->u.l.next = g->uvhead.u.l.next;
	uv->u.l.next->u.l.prev = uv;
	g->uvhead.u.l.next = uv;
	lua_assert(uv->u.l.next->u.l.prev == uv && uv->u.l.prev->u.l.next == uv);
	return uv;
}


static void unlinkupval (UpVal *uv) {
	lua_assert(uv->u.l.next->u.l.prev == uv && uv->u.l.prev->u.l.next == uv);
	uv->u.l.next->u.l.prev = uv->u.l.prev;  /* remove from `uvhead' list */
	uv->u.l.prev->u.l.next = uv->u.l.next;
}


void luaF_freeupval (lua_State *L, UpVal *uv) {
	if (uv->v != &uv->u.value)  /* is it open? */
		unlinkupval(uv);  /* remove from open list */
	luaM_free(L, uv);  /* free upvalue */
}


void luaF_close (lua_State *L, StkId level) {
	UpVal *uv;
	global_State *g = G(L);
	while (L->openupval != NULL && (uv = ngcotouv(L->openupval))->v >= level) {
		GCObject *o = obj2gco(uv);
		lua_assert(!isblack(o) && uv->v != &uv->u.value);
		L->openupval = uv->next;  /* remove from `open' list */
		if (isdead(g, o))
			luaF_freeupval(L, uv);  /* free upvalue */
		else {
			unlinkupval(uv);
			setobj(L, &uv->u.value, uv->v);
			uv->v = &uv->u.value;  /* now current value lives here */
			luaC_linkupval(L, uv);  /* link upvalue into `gcroot' list */
		}
	}
}


Proto *luaF_newproto (lua_State *L) {
	Proto *f = luaM_new(L, Proto);
	luaC_link(L, obj2gco(f), LUA_TPROTO);
	f->k = NULL;
	f->sizek = 0;
	f->p = NULL;
	f->sizep = 0;
	f->code = NULL;
	f->sizecode = 0;
	f->sizelineinfo = 0;
	f->sizeupvalues = 0;
	f->nups = 0;
	f->upvalues = NULL;
	f->numparams = 0;
	f->is_vararg = 0;
	f->maxstacksize = 0;
	f->lineinfo = NULL;
	f->sizelocvars = 0;
	f->locvars = NULL;
	f->linedefined = 0;
	f->lastlinedefined = 0;
	f->source = NULL;
	return f;
}


void luaF_freeproto (lua_State *L, Proto *f) {
	luaM_freearray(L, f->code, f->sizecode, Instruction);
	luaM_freearray(L, f->p, f->sizep, Proto *);
	luaM_freearray(L, f->k, f->sizek, TValue);
	luaM_freearray(L, f->lineinfo, f->sizelineinfo, int);
	luaM_freearray(L, f->locvars, f->sizelocvars, struct LocVar);
	luaM_freearray(L, f->upvalues, f->sizeupvalues, TString *);
	luaM_free(L, f);
}


void luaF_freeclosure (lua_State *L, Closure *c) {
	int size = (c->c.isC) ? sizeCclosure(c->c.nupvalues) :
		sizeLclosure(c->l.nupvalues);
	luaM_freemem(L, c, size);
}


/*
** Look for n-th local variable at line `line' in function `func'.
** Returns NULL if not found.
*/
const char *luaF_getlocalname (const Proto *f, int local_number, int pc) {
	int i;
	for (i = 0; i<f->sizelocvars && f->locvars[i].startpc <= pc; i++) {
		if (pc < f->locvars[i].endpc) {  /* is variable active? */
			local_number--;
			if (local_number == 0)
				return getstr(f->locvars[i].varname);
		}
	}
	return NULL;  /* not found */
}


//-------------------------------------------------------------lstring.c----------------------------------------------------
void luaS_resize (lua_State *L, int newsize) {
	GCObject **newhash;
	stringtable *tb;
	int i;
	if (G(L)->gcstate == GCSsweepstring)
		return;  /* cannot resize during GC traverse */
	newhash = luaM_newvector(L, newsize, GCObject *);
	tb = &G(L)->strt;
	for (i=0; i<newsize; i++) newhash[i] = NULL;
	/* rehash */
	for (i=0; i<tb->size; i++) {
		GCObject *p = tb->hash[i];
		while (p) {  /* for each node in the list */
			GCObject *next = p->gch.next;  /* save next */
			unsigned int h = gco2ts(p)->hash;
			int h1 = lmod(h, newsize);  /* new position */
			lua_assert(cast_int(h%newsize) == lmod(h, newsize));
			p->gch.next = newhash[h1];  /* chain it */
			newhash[h1] = p;
			p = next;
		}
	}
	luaM_freearray(L, tb->hash, tb->size, TString *);
	tb->size = newsize;
	tb->hash = newhash;
}


static TString *newlstr (lua_State *L, const char *str, size_t l, unsigned int h) 
{
	 TString *ts;
	 stringtable *tb;
	 if (l+1 > (MAX_SIZET - sizeof(TString))/sizeof(char))
		 luaM_toobig(L);
	 ts = cast(TString *, luaM_malloc(L, (l+1)*sizeof(char)+sizeof(TString)));
	 ts->tsv.len = l;
	 ts->tsv.hash = h;
	 ts->tsv.marked = luaC_white(G(L));
	 ts->tsv.tt = LUA_TSTRING;
	 ts->tsv.reserved = 0;
	 memcpy(ts+1, str, l*sizeof(char));
	 ((char *)(ts+1))[l] = '\0';  /* ending 0 */
	 tb = &G(L)->strt;
	 h = lmod(h, tb->size);
	 ts->tsv.next = tb->hash[h];  /* chain new entry */
	 tb->hash[h] = obj2gco(ts);
	 tb->nuse++;
	 if (tb->nuse > cast(lu_int32, tb->size) && tb->size <= MAX_INT/2)
		 luaS_resize(L, tb->size*2);  /* too crowded */
	 return ts;
}


TString *luaS_newlstr (lua_State *L, const char *str, size_t l) {
	GCObject *o;
	unsigned int h = cast(unsigned int, l);  /* seed */
	size_t step = (l>>5)+1;  /* if string is too long, don't hash all its chars */
	size_t l1;
	for (l1=l; l1>=step; l1-=step)  /* compute hash */
		h = h ^ ((h<<5)+(h>>2)+cast(unsigned char, str[l1-1]));
	for (o = G(L)->strt.hash[lmod(h, G(L)->strt.size)];
		o != NULL;
		o = o->gch.next) {
			TString *ts = rawgco2ts(o);
			if (ts->tsv.len == l && (memcmp(str, getstr(ts), l) == 0)) {
				/* string may be dead */
				if (isdead(G(L), o)) changewhite(o);
				return ts;
			}
	}
	return newlstr(L, str, l, h);  /* not found */
}


Udata *luaS_newudata (lua_State *L, size_t s, Table *e) {
	Udata *u;
	if (s > MAX_SIZET - sizeof(Udata))
		luaM_toobig(L);
	u = cast(Udata *, luaM_malloc(L, s + sizeof(Udata)));
	u->uv.marked = luaC_white(G(L));  /* is not finalized */
	u->uv.tt = LUA_TUSERDATA;
	u->uv.len = s;
	u->uv.metatable = NULL;
	u->uv.env = e;
	/* chain it on udata list (after main thread) */
	u->uv.next = G(L)->mainthread->next;
	G(L)->mainthread->next = obj2gco(u);
	return u;
}

//-------------------------------------------------------------lopcodes.c----------------------------------------------------
/* ORDER OP */

const char *const luaP_opnames[NUM_OPCODES+1] = {
	"MOVE",
	"LOADK",
	"LOADBOOL",
	"LOADNIL",
	"GETUPVAL",
	"GETGLOBAL",
	"GETTABLE",
	"SETGLOBAL",
	"SETUPVAL",
	"SETTABLE",
	"NEWTABLE",
	"SELF",
	"ADD",
	"SUB",
	"MUL",
	"DIV",
	"MOD",
	"POW",
	"UNM",
	"NOT",
	"LEN",
	"CONCAT",
	"JMP",
	"EQ",
	"LT",
	"LE",
	"TEST",
	"TESTSET",
	"CALL",
	"TAILCALL",
	"RETURN",
	"FORLOOP",
	"FORPREP",
	"TFORLOOP",
	"SETLIST",
	"CLOSE",
	"CLOSURE",
	"VARARG",
	NULL
};


#define opmode(t,a,b,c,m) (((t)<<7) | ((a)<<6) | ((b)<<4) | ((c)<<2) | (m))

const lu_byte luaP_opmodes[NUM_OPCODES] = {
	/*       T  A    B       C     mode		   opcode	*/
	opmode(0, 1, OpArgR, OpArgN, iABC) 		/* OP_MOVE */
	,opmode(0, 1, OpArgK, OpArgN, iABx)		/* OP_LOADK */
	,opmode(0, 1, OpArgU, OpArgU, iABC)		/* OP_LOADBOOL */
	,opmode(0, 1, OpArgR, OpArgN, iABC)		/* OP_LOADNIL */
	,opmode(0, 1, OpArgU, OpArgN, iABC)		/* OP_GETUPVAL */
	,opmode(0, 1, OpArgK, OpArgN, iABx)		/* OP_GETGLOBAL */
	,opmode(0, 1, OpArgR, OpArgK, iABC)		/* OP_GETTABLE */
	,opmode(0, 0, OpArgK, OpArgN, iABx)		/* OP_SETGLOBAL */
	,opmode(0, 0, OpArgU, OpArgN, iABC)		/* OP_SETUPVAL */
	,opmode(0, 0, OpArgK, OpArgK, iABC)		/* OP_SETTABLE */
	,opmode(0, 1, OpArgU, OpArgU, iABC)		/* OP_NEWTABLE */
	,opmode(0, 1, OpArgR, OpArgK, iABC)		/* OP_SELF */
	,opmode(0, 1, OpArgK, OpArgK, iABC)		/* OP_ADD */
	,opmode(0, 1, OpArgK, OpArgK, iABC)		/* OP_SUB */
	,opmode(0, 1, OpArgK, OpArgK, iABC)		/* OP_MUL */
	,opmode(0, 1, OpArgK, OpArgK, iABC)		/* OP_DIV */
	,opmode(0, 1, OpArgK, OpArgK, iABC)		/* OP_MOD */
	,opmode(0, 1, OpArgK, OpArgK, iABC)		/* OP_POW */
	,opmode(0, 1, OpArgR, OpArgN, iABC)		/* OP_UNM */
	,opmode(0, 1, OpArgR, OpArgN, iABC)		/* OP_NOT */
	,opmode(0, 1, OpArgR, OpArgN, iABC)		/* OP_LEN */
	,opmode(0, 1, OpArgR, OpArgR, iABC)		/* OP_CONCAT */
	,opmode(0, 0, OpArgR, OpArgN, iAsBx)		/* OP_JMP */
	,opmode(1, 0, OpArgK, OpArgK, iABC)		/* OP_EQ */
	,opmode(1, 0, OpArgK, OpArgK, iABC)		/* OP_LT */
	,opmode(1, 0, OpArgK, OpArgK, iABC)		/* OP_LE */
	,opmode(1, 1, OpArgR, OpArgU, iABC)		/* OP_TEST */
	,opmode(1, 1, OpArgR, OpArgU, iABC)		/* OP_TESTSET */
	,opmode(0, 1, OpArgU, OpArgU, iABC)		/* OP_CALL */
	,opmode(0, 1, OpArgU, OpArgU, iABC)		/* OP_TAILCALL */
	,opmode(0, 0, OpArgU, OpArgN, iABC)		/* OP_RETURN */
	,opmode(0, 1, OpArgR, OpArgN, iAsBx)		/* OP_FORLOOP */
	,opmode(0, 1, OpArgR, OpArgN, iAsBx)		/* OP_FORPREP */
	,opmode(1, 0, OpArgN, OpArgU, iABC)		/* OP_TFORLOOP */
	,opmode(0, 0, OpArgU, OpArgU, iABC)		/* OP_SETLIST */
	,opmode(0, 0, OpArgN, OpArgN, iABC)		/* OP_CLOSE */
	,opmode(0, 1, OpArgU, OpArgN, iABx)		/* OP_CLOSURE */
	,opmode(0, 1, OpArgU, OpArgN, iABC)		/* OP_VARARG */
};

//-------------------------------------------------------------debug.c-------------------------------------------------------

//from object.h
LUAI_FUNC int luaO_log2 (unsigned int x);
LUAI_FUNC int luaO_int2fb (unsigned int x);
LUAI_FUNC int luaO_fb2int (int x);
LUAI_FUNC int luaO_rawequalObj (const TValue *t1, const TValue *t2);
LUAI_FUNC int luaO_str2d (const char *s, lua_Number *result);
LUAI_FUNC const char *luaO_pushvfstring (lua_State *L, const char *fmt,
										 va_list argp);
LUAI_FUNC const char *luaO_pushfstring (lua_State *L, const char *fmt, ...);
LUAI_FUNC void luaO_chunkid (char *out, const char *source, size_t len);

//from table.h
LUAI_FUNC const TValue *luaH_getnum (Table *t, int key);
LUAI_FUNC TValue *luaH_setnum (lua_State *L, Table *t, int key);
LUAI_FUNC const TValue *luaH_getstr (Table *t, TString *key);
LUAI_FUNC TValue *luaH_setstr (lua_State *L, Table *t, TString *key);
LUAI_FUNC const TValue *luaH_get (Table *t, const TValue *key);
LUAI_FUNC TValue *luaH_set (lua_State *L, Table *t, const TValue *key);
LUAI_FUNC Table *luaH_new (lua_State *L, int narray, int lnhash);
LUAI_FUNC void luaH_resizearray (lua_State *L, Table *t, int nasize);
LUAI_FUNC void luaH_free (lua_State *L, Table *t);
LUAI_FUNC int luaH_next (lua_State *L, Table *t, StkId key);
LUAI_FUNC int luaH_getn (Table *t);


#if defined(LUA_DEBUG)
LUAI_FUNC Node *luaH_mainposition (const Table *t, const TValue *key);
LUAI_FUNC int luaH_isdummy (Node *n);
#endif

//from lvm.h
LUAI_FUNC int luaV_lessthan (lua_State *L, const TValue *l, const TValue *r);
LUAI_FUNC int luaV_equalval (lua_State *L, const TValue *t1, const TValue *t2);
LUAI_FUNC const TValue *luaV_tonumber (const TValue *obj, TValue *n);
LUAI_FUNC int luaV_tostring (lua_State *L, StkId obj);
LUAI_FUNC void luaV_gettable (lua_State *L, const TValue *t, TValue *key,
							  StkId val);
LUAI_FUNC void luaV_settable (lua_State *L, const TValue *t, TValue *key,
							  StkId val);
LUAI_FUNC void luaV_execute (lua_State *L, int nexeccalls);
LUAI_FUNC void luaV_concat (lua_State *L, int total, int last);

//from lapi.h
LUAI_FUNC void luaA_pushobject (lua_State *L, const TValue *o);

static const char *getfuncname (lua_State *L, CallInfo *ci, const char **name);

static int currentpc (lua_State *L, CallInfo *ci) {
	if (!isLua(ci)) return -1;  /* function is not a Lua function? */
	if (ci == L->ci)
		ci->savedpc = L->savedpc;
	return pcRel(ci->savedpc, ci_func(ci)->l.p);
}


static int currentline (lua_State *L, CallInfo *ci) {
	int pc = currentpc(L, ci);
	if (pc < 0)
		return -1;  /* only active lua functions have current-line information */
	else
		return getline(ci_func(ci)->l.p, pc);
}


/*
** this function can be called asynchronous (e.g. during a signal)
*/
LUA_API int lua_sethook (lua_State *L, lua_Hook func, int mask, int count) {
	if (func == NULL || mask == 0) {  /* turn off hooks? */
		mask = 0;
		func = NULL;
	}
	L->hook = func;
	L->basehookcount = count;
	resethookcount(L);
	L->hookmask = cast_byte(mask);
	return 1;
}


LUA_API lua_Hook lua_gethook (lua_State *L) {
	return L->hook;
}


LUA_API int lua_gethookmask (lua_State *L) {
	return L->hookmask;
}


LUA_API int lua_gethookcount (lua_State *L) {
	return L->basehookcount;
}


LUA_API int lua_getstack (lua_State *L, int level, lua_Debug *ar) {
	int status;
	CallInfo *ci;
	lua_lock(L);
	for (ci = L->ci; level > 0 && ci > L->base_ci; ci--) {
		level--;
		if (f_isLua(ci))  /* Lua function? */
			level -= ci->tailcalls;  /* skip lost tail calls */
	}
	if (level == 0 && ci > L->base_ci) {  /* level found? */
		status = 1;
		ar->i_ci = cast_int(ci - L->base_ci);
	}
	else if (level < 0) {  /* level is of a lost tail call? */
		status = 1;
		ar->i_ci = 0;
	}
	else status = 0;  /* no such level */
	lua_unlock(L);
	return status;
}


static Proto *getluaproto (CallInfo *ci) {
	return (isLua(ci) ? ci_func(ci)->l.p : NULL);
}


static const char *findlocal (lua_State *L, CallInfo *ci, int n) {
	const char *name;
	Proto *fp = getluaproto(ci);
	if (fp && (name = luaF_getlocalname(fp, n, currentpc(L, ci))) != NULL)
		return name;  /* is a local variable in a Lua function */
	else {
		StkId limit = (ci == L->ci) ? L->top : (ci+1)->func;
		if (limit - ci->base >= n && n > 0)  /* is 'n' inside 'ci' stack? */
			return "(*temporary)";
		else
			return NULL;
	}
}


LUA_API const char *lua_getlocal (lua_State *L, const lua_Debug *ar, int n) {
	CallInfo *ci = L->base_ci + ar->i_ci;
	const char *name = findlocal(L, ci, n);
	lua_lock(L);
	if (name)
		luaA_pushobject(L, ci->base + (n - 1));
	lua_unlock(L);
	return name;
}


LUA_API const char *lua_setlocal (lua_State *L, const lua_Debug *ar, int n) {
	CallInfo *ci = L->base_ci + ar->i_ci;
	const char *name = findlocal(L, ci, n);
	lua_lock(L);
	if (name)
		setobjs2s(L, ci->base + (n - 1), L->top - 1);
	L->top--;  /* pop value */
	lua_unlock(L);
	return name;
}


static void funcinfo (lua_Debug *ar, Closure *cl) {
	if (cl->c.isC) {
		ar->source = "=[C]";
		ar->linedefined = -1;
		ar->lastlinedefined = -1;
		ar->what = "C";
	}
	else {
		ar->source = getstr(cl->l.p->source);
		ar->linedefined = cl->l.p->linedefined;
		ar->lastlinedefined = cl->l.p->lastlinedefined;
		ar->what = (ar->linedefined == 0) ? "main" : "Lua";
	}
	luaO_chunkid(ar->short_src, ar->source, LUA_IDSIZE);
}


static void info_tailcall (lua_Debug *ar) {
	ar->name = ar->namewhat = "";
	ar->what = "tail";
	ar->lastlinedefined = ar->linedefined = ar->currentline = -1;
	ar->source = "=(tail call)";
	luaO_chunkid(ar->short_src, ar->source, LUA_IDSIZE);
	ar->nups = 0;
}


static void collectvalidlines (lua_State *L, Closure *f) {
	if (f == NULL || f->c.isC) {
		setnilvalue(L->top);
	}
	else {
		Table *t = luaH_new(L, 0, 0);
		int *lineinfo = f->l.p->lineinfo;
		int i;
		for (i=0; i<f->l.p->sizelineinfo; i++)
			setbvalue(luaH_setnum(L, t, lineinfo[i]), 1);
		sethvalue(L, L->top, t); 
	}
	incr_top(L);
}


static int auxgetinfo (lua_State *L, const char *what, lua_Debug *ar, Closure *f, CallInfo *ci) {
						   int status = 1;
						   if (f == NULL) {
							   info_tailcall(ar);
							   return status;
						   }
						   for (; *what; what++) {
							   switch (*what) {
	  case 'S': {
		  funcinfo(ar, f);
		  break;
				}
	  case 'l': {
		  ar->currentline = (ci) ? currentline(L, ci) : -1;
		  break;
				}
	  case 'u': {
		  ar->nups = f->c.nupvalues;
		  break;
				}
	  case 'n': {
		  ar->namewhat = (ci) ? getfuncname(L, ci, &ar->name) : NULL;
		  if (ar->namewhat == NULL) {
			  ar->namewhat = "";  /* not found */
			  ar->name = NULL;
		  }
		  break;
				}
	  case 'L':
	  case 'f':  /* handled by lua_getinfo */
		  break;
	  default: status = 0;  /* invalid option */
							   }
						   }
						   return status;
}


LUA_API int lua_getinfo (lua_State *L, const char *what, lua_Debug *ar) {
	int status;
	Closure *f = NULL;
	CallInfo *ci = NULL;
	lua_lock(L);
	if (*what == '>') {
		StkId func = L->top - 1;
		luai_apicheck(L, ttisfunction(func));
		what++;  /* skip the '>' */
		f = clvalue(func);
		L->top--;  /* pop function */
	}
	else if (ar->i_ci != 0) {  /* no tail call? */
		ci = L->base_ci + ar->i_ci;
		lua_assert(ttisfunction(ci->func));
		f = clvalue(ci->func);
	}
	status = auxgetinfo(L, what, ar, f, ci);
	if (strchr(what, 'f')) {
		if (f == NULL) setnilvalue(L->top);
		else setclvalue(L, L->top, f);
		incr_top(L);
	}
	if (strchr(what, 'L'))
		collectvalidlines(L, f);
	lua_unlock(L);
	return status;
}


/*
** {======================================================
** Symbolic Execution and code checker
** =======================================================
*/

#define check(x)		if (!(x)) return 0;

#define checkjump(pt,pc)	check(0 <= pc && pc < pt->sizecode)

#define checkreg(pt,reg)	check((reg) < (pt)->maxstacksize)



static int precheck (const Proto *pt) {
	check(pt->maxstacksize <= MAXSTACK);
	lua_assert(pt->numparams+(pt->is_vararg & VARARG_HASARG) <= pt->maxstacksize);
	lua_assert(!(pt->is_vararg & VARARG_NEEDSARG) ||
		(pt->is_vararg & VARARG_HASARG));
	check(pt->sizeupvalues <= pt->nups);
	check(pt->sizelineinfo == pt->sizecode || pt->sizelineinfo == 0);
	check(GET_OPCODE(pt->code[pt->sizecode-1]) == OP_RETURN);
	return 1;
}


#define checkopenop(pt,pc)	luaG_checkopenop((pt)->code[(pc)+1])

int luaG_checkopenop (Instruction i) {
	switch (GET_OPCODE(i)) {
	case OP_CALL:
	case OP_TAILCALL:
	case OP_RETURN:
	case OP_SETLIST: {
		check(GETARG_B(i) == 0);
		return 1;
					 }
	default: return 0;  /* invalid instruction after an open call */
	}
}


static int checkArgMode (const Proto *pt, int r, enum OpArgMask mode) {
	switch (mode) {
	case OpArgN: check(r == 0); break;
	case OpArgU: break;
	case OpArgR: checkreg(pt, r); break;
	case OpArgK:
		check(ISK(r) ? INDEXK(r) < pt->sizek : r < pt->maxstacksize);
		break;
	}
	return 1;
}


static Instruction symbexec (const Proto *pt, int lastpc, int reg) {
	int pc;
	int last;  /* stores position of last instruction that changed `reg' */
	last = pt->sizecode-1;  /* points to final return (a `neutral' instruction) */
	check(precheck(pt));
	for (pc = 0; pc < lastpc; pc++) {
		Instruction i = pt->code[pc];
		OpCode op = GET_OPCODE(i);
		int a = GETARG_A(i);
		int b = 0;
		int c = 0;
		check(op < NUM_OPCODES);
		checkreg(pt, a);
		switch (getOpMode(op)) {
	  case iABC: {
		  b = GETARG_B(i);
		  c = GETARG_C(i);
		  check(checkArgMode(pt, b, getBMode(op)));
		  check(checkArgMode(pt, c, getCMode(op)));
		  break;
				 }
	  case iABx: {
		  b = GETARG_Bx(i);
		  if (getBMode(op) == OpArgK) check(b < pt->sizek);
		  break;
				 }
	  case iAsBx: {
		  b = GETARG_sBx(i);
		  if (getBMode(op) == OpArgR) {
			  int dest = pc+1+b;
			  check(0 <= dest && dest < pt->sizecode);
			  if (dest > 0) {
				  /* cannot jump to a setlist count */
				  Instruction d = pt->code[dest-1];
				  check(!(GET_OPCODE(d) == OP_SETLIST && GETARG_C(d) == 0));
			  }
		  }
		  break;
				  }
		}
		if (testAMode(op)) {
			if (a == reg) last = pc;  /* change register `a' */
		}
		if (testTMode(op)) {
			check(pc+2 < pt->sizecode);  /* check skip */
			check(GET_OPCODE(pt->code[pc+1]) == OP_JMP);
		}
		switch (op) {
	  case OP_LOADBOOL: {
		  check(c == 0 || pc+2 < pt->sizecode);  /* check its jump */
		  break;
						}
	  case OP_LOADNIL: {
		  if (a <= reg && reg <= b)
			  last = pc;  /* set registers from `a' to `b' */
		  break;
					   }
	  case OP_GETUPVAL:
	  case OP_SETUPVAL: {
		  check(b < pt->nups);
		  break;
						}
	  case OP_GETGLOBAL:
	  case OP_SETGLOBAL: {
		  check(ttisstring(&pt->k[b]));
		  break;
						 }
	  case OP_SELF: {
		  checkreg(pt, a+1);
		  if (reg == a+1) last = pc;
		  break;
					}
	  case OP_CONCAT: {
		  check(b < c);  /* at least two operands */
		  break;
					  }
	  case OP_TFORLOOP: {
		  check(c >= 1);  /* at least one result (control variable) */
		  checkreg(pt, a+2+c);  /* space for results */
		  if (reg >= a+2) last = pc;  /* affect all regs above its base */
		  break;
						}
	  case OP_FORLOOP:
	  case OP_FORPREP:
		  checkreg(pt, a+3);
		  /* go through */
	  case OP_JMP: {
		  int dest = pc+1+b;
		  /* not full check and jump is forward and do not skip `lastpc'? */
		  if (reg != NO_REG && pc < dest && dest <= lastpc)
			  pc += b;  /* do the jump */
		  break;
				   }
	  case OP_CALL:
	  case OP_TAILCALL: {
		  if (b != 0) {
			  checkreg(pt, a+b-1);
		  }
		  c--;  /* c = num. returns */
		  if (c == LUA_MULTRET) {
			  check(checkopenop(pt, pc));
		  }
		  else if (c != 0)
			  checkreg(pt, a+c-1);
		  if (reg >= a) last = pc;  /* affect all registers above base */
		  break;
						}
	  case OP_RETURN: {
		  b--;  /* b = num. returns */
		  if (b > 0) checkreg(pt, a+b-1);
		  break;
					  }
	  case OP_SETLIST: {
		  if (b > 0) checkreg(pt, a + b);
		  if (c == 0) pc++;
		  break;
					   }
	  case OP_CLOSURE: {
		  int nup, j;
		  check(b < pt->sizep);
		  nup = pt->p[b]->nups;
		  check(pc + nup < pt->sizecode);
		  for (j = 1; j <= nup; j++) {
			  OpCode op1 = GET_OPCODE(pt->code[pc + j]);
			  check(op1 == OP_GETUPVAL || op1 == OP_MOVE);
		  }
		  if (reg != NO_REG)  /* tracing? */
			  pc += nup;  /* do not 'execute' these pseudo-instructions */
		  break;
					   }
	  case OP_VARARG: {
		  check((pt->is_vararg & VARARG_ISVARARG) &&
			  !(pt->is_vararg & VARARG_NEEDSARG));
		  b--;
		  if (b == LUA_MULTRET) check(checkopenop(pt, pc));
		  checkreg(pt, a+b-1);
		  break;
					  }
	  default: break;
		}
	}
	return pt->code[last];
}

#undef check
#undef checkjump
#undef checkreg

/* }====================================================== */


int luaG_checkcode (const Proto *pt) {
	return (symbexec(pt, pt->sizecode, NO_REG) != 0);
}


static const char *kname (Proto *p, int c) {
	if (ISK(c) && ttisstring(&p->k[INDEXK(c)]))
		return svalue(&p->k[INDEXK(c)]);
	else
		return "?";
}


static const char *getobjname (lua_State *L, CallInfo *ci, int stackpos, const char **name) {
								   if (isLua(ci)) {  /* a Lua function? */
									   Proto *p = ci_func(ci)->l.p;
									   int pc = currentpc(L, ci);
									   Instruction i;
									   *name = luaF_getlocalname(p, stackpos+1, pc);
									   if (*name)  /* is a local? */
										   return "local";
									   i = symbexec(p, pc, stackpos);  /* try symbolic execution */
									   lua_assert(pc != -1);
									   switch (GET_OPCODE(i)) {
	  case OP_GETGLOBAL: {
		  int g = GETARG_Bx(i);  /* global index */
		  lua_assert(ttisstring(&p->k[g]));
		  *name = svalue(&p->k[g]);
		  return "global";
						 }
	  case OP_MOVE: {
		  int a = GETARG_A(i);
		  int b = GETARG_B(i);  /* move from `b' to `a' */
		  if (b < a)
			  return getobjname(L, ci, b, name);  /* get name for `b' */
		  break;
					}
	  case OP_GETTABLE: {
		  int k = GETARG_C(i);  /* key index */
		  *name = kname(p, k);
		  return "field";
						}
	  case OP_GETUPVAL: {
		  int u = GETARG_B(i);  /* upvalue index */
		  *name = p->upvalues ? getstr(p->upvalues[u]) : "?";
		  return "upvalue";
						}
	  case OP_SELF: {
		  int k = GETARG_C(i);  /* key index */
		  *name = kname(p, k);
		  return "method";
					}
	  default: break;
									   }
								   }
								   return NULL;  /* no useful name found */
}


static const char *getfuncname (lua_State *L, CallInfo *ci, const char **name) {
	Instruction i;
	if ((isLua(ci) && ci->tailcalls > 0) || !isLua(ci - 1))
		return NULL;  /* calling function is not Lua (or is unknown) */
	ci--;  /* calling function */
	i = ci_func(ci)->l.p->code[currentpc(L, ci)];
	if (GET_OPCODE(i) == OP_CALL || GET_OPCODE(i) == OP_TAILCALL ||
		GET_OPCODE(i) == OP_TFORLOOP)
		return getobjname(L, ci, GETARG_A(i), name);
	else
		return NULL;  /* no useful name can be found */
}


/* only ANSI way to check whether a pointer points to an array */
static int isinstack (CallInfo *ci, const TValue *o) {
	StkId p;
	for (p = ci->base; p < ci->top; p++)
		if (o == p) return 1;
	return 0;
}


static void addinfo (lua_State *L, const char *msg) {
	CallInfo *ci = L->ci;
	if (isLua(ci)) {  /* is Lua code? */
		char buff[LUA_IDSIZE];  /* add file:line information */
		int line = currentline(L, ci);
		luaO_chunkid(buff, getstr(getluaproto(ci)->source), LUA_IDSIZE);
		luaO_pushfstring(L, "%s:%d: %s", buff, line, msg);
	}
}
void luaG_errormsg (lua_State *L) {
	if (L->errfunc != 0) {  /* is there an error handling function? */
		StkId errfunc = restorestack(L, L->errfunc);
		if (!ttisfunction(errfunc)) luaD_throw(L, LUA_ERRERR);
		setobjs2s(L, L->top, L->top - 1);  /* move argument */
		setobjs2s(L, L->top - 1, errfunc);  /* push function */
		incr_top(L);
		luaD_call(L, L->top - 2, 1);  /* call it */
	}
	luaD_throw(L, LUA_ERRRUN);
}



void luaG_runerror (lua_State *L, const char *fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	addinfo(L, luaO_pushvfstring(L, fmt, argp));
	va_end(argp);
	luaG_errormsg(L);
}

void luaG_typeerror (lua_State *L, const TValue *o, const char *op) {
	const char *name = NULL;
	const char *t = luaT_typenames[ttype(o)];
	const char *kind = (isinstack(L->ci, o)) ?
		getobjname(L, L->ci, cast_int(o - L->base), &name) :
	NULL;
	if (kind)
		luaG_runerror(L, "attempt to %s %s " LUA_QS " (a %s value)",
		op, kind, name, t);
	else
		luaG_runerror(L, "attempt to %s a %s value", op, t);
}


void luaG_concaterror (lua_State *L, StkId p1, StkId p2) {
	if (ttisstring(p1)) p1 = p2;
	lua_assert(!ttisstring(p1));
	luaG_typeerror(L, p1, "concatenate");
}


void luaG_aritherror (lua_State *L, const TValue *p1, const TValue *p2) {
	TValue temp;
	if (luaV_tonumber(p1, &temp) == NULL)
		p2 = p1;  /* first operand is wrong */
	luaG_typeerror(L, p2, "perform arithmetic on");
}


int luaG_ordererror (lua_State *L, const TValue *p1, const TValue *p2) {
	const char *t1 = luaT_typenames[ttype(p1)];
	const char *t2 = luaT_typenames[ttype(p2)];
	if (t1[2] == t2[2])
		luaG_runerror(L, "attempt to compare two %s values", t1);
	else
		luaG_runerror(L, "attempt to compare %s with %s", t1, t2);
	return 0;
}

//-------------------------------------------------------------lzio.c-------------------------------------------------------
int luaZ_fill (ZIO *z) {
	size_t size;
	lua_State *L = z->L;
	const char *buff;
	lua_unlock(L);
	buff = z->reader(L, z->data, &size);
	lua_lock(L);
	if (buff == NULL || size == 0) return EOZ;
	z->n = size - 1;
	z->p = buff;
	return char2int(*(z->p++));
}


int luaZ_lookahead (ZIO *z) {
	if (z->n == 0) {
		if (luaZ_fill(z) == EOZ)
			return EOZ;
		else {
			z->n++;  /* luaZ_fill removed first byte; put back it */
			z->p--;
		}
	}
	return char2int(*z->p);
}


void luaZ_init (lua_State *L, ZIO *z, lua_Reader reader, void *data) {
	z->L = L;
	z->reader = reader;
	z->data = data;
	z->n = 0;
	z->p = NULL;
}


/* --------------------------------------------------------------- read --- */
size_t luaZ_read (ZIO *z, void *b, size_t n) {
	while (n) {
		size_t m;
		if (luaZ_lookahead(z) == EOZ)
			return n;  /* return number of missing bytes */
		m = (n <= z->n) ? n : z->n;  /* min. between n and z->n */
		memcpy(b, z->p, m);
		z->n -= m;
		z->p += m;
		b = (char *)b + m;
		n -= m;
	}
	return 0;
}

/* ------------------------------------------------------------------------ */
char *luaZ_openspace (lua_State *L, Mbuffer *buff, size_t n) {
	if (n > buff->buffsize) {
		if (n < LUA_MINBUFFER) n = LUA_MINBUFFER;
		luaZ_resizebuffer(L, buff, n);
	}
	return buff->buffer;
}

//-------------------------------------------------------------ldo.c--------------------------------------------------------

/* load one chunk; from lundump.c */
LUAI_FUNC Proto* luaU_undump (lua_State* L, ZIO* Z, Mbuffer* buff, const char* name);
LUAI_FUNC Proto *luaY_parser (lua_State *L, ZIO *z, Mbuffer *buff, const char *name);

/*
** {======================================================
** Error-recovery functions
** =======================================================
*/


/* chain list of long jump buffers */
struct lua_longjmp {
	struct lua_longjmp *previous;
	luai_jmpbuf b;
	volatile int status;  /* error code */
};


void luaD_seterrorobj (lua_State *L, int errcode, StkId oldtop) {
	switch (errcode) {
	case LUA_ERRMEM: {
		setsvalue2s(L, oldtop, luaS_newliteral(L, MEMERRMSG));
		break;
					 }
	case LUA_ERRERR: {
		setsvalue2s(L, oldtop, luaS_newliteral(L, "error in error handling"));
		break;
					 }
	case LUA_ERRSYNTAX:
	case LUA_ERRRUN: {
		setobjs2s(L, oldtop, L->top - 1);  /* error message on current top */
		break;
					 }
	}
	L->top = oldtop + 1;
}


static void restore_stack_limit (lua_State *L) {
	lua_assert(L->stack_last - L->stack == L->stacksize - EXTRA_STACK - 1);
	if (L->size_ci > LUAI_MAXCALLS) {  /* there was an overflow? */
		int inuse = cast_int(L->ci - L->base_ci);
		if (inuse + 1 < LUAI_MAXCALLS)  /* can `undo' overflow? */
			luaD_reallocCI(L, LUAI_MAXCALLS);
	}
}


static void resetstack (lua_State *L, int status) {
	L->ci = L->base_ci;
	L->base = L->ci->base;
	luaF_close(L, L->base);  /* close eventual pending closures */
	luaD_seterrorobj(L, status, L->base);
	L->nCcalls = 0;
	L->allowhook = 1;
	restore_stack_limit(L);
	L->errfunc = 0;
	L->errorJmp = NULL;
}


void luaD_throw (lua_State *L, int errcode) {
	if (L->errorJmp) {
		L->errorJmp->status = errcode;
		LUAI_THROW(L, L->errorJmp);
	}
	else {
		L->status = cast_byte(errcode);
		if (G(L)->panic) {
			resetstack(L, errcode);
			lua_unlock(L);
			G(L)->panic(L);
		}
		exit(EXIT_FAILURE);
	}
}


int luaD_rawrunprotected (lua_State *L, Pfunc f, void *ud) {
	struct lua_longjmp lj;
	lj.status = 0;
	lj.previous = L->errorJmp;  /* chain new error handler */
	L->errorJmp = &lj;
	LUAI_TRY(L, &lj,
		(*f)(L, ud);
	);
	L->errorJmp = lj.previous;  /* restore old error handler */
	return lj.status;
}

/* }====================================================== */


static void correctstack (lua_State *L, TValue *oldstack) {
	CallInfo *ci;
	GCObject *up;
	L->top = (L->top - oldstack) + L->stack;
	for (up = L->openupval; up != NULL; up = up->gch.next)
		gco2uv(up)->v = (gco2uv(up)->v - oldstack) + L->stack;
	for (ci = L->base_ci; ci <= L->ci; ci++) {
		ci->top = (ci->top - oldstack) + L->stack;
		ci->base = (ci->base - oldstack) + L->stack;
		ci->func = (ci->func - oldstack) + L->stack;
	}
	L->base = (L->base - oldstack) + L->stack;
}


void luaD_reallocstack (lua_State *L, int newsize) {
	TValue *oldstack = L->stack;
	int realsize = newsize + 1 + EXTRA_STACK;
	lua_assert(L->stack_last - L->stack == L->stacksize - EXTRA_STACK - 1);
	luaM_reallocvector(L, L->stack, L->stacksize, realsize, TValue);
	L->stacksize = realsize;
	L->stack_last = L->stack+newsize;
	correctstack(L, oldstack);
}


void luaD_reallocCI (lua_State *L, int newsize) {
	CallInfo *oldci = L->base_ci;
	luaM_reallocvector(L, L->base_ci, L->size_ci, newsize, CallInfo);
	L->size_ci = newsize;
	L->ci = (L->ci - oldci) + L->base_ci;
	L->end_ci = L->base_ci + L->size_ci - 1;
}


void luaD_growstack (lua_State *L, int n) {
	if (n <= L->stacksize)  /* double size is enough? */
		luaD_reallocstack(L, 2*L->stacksize);
	else
		luaD_reallocstack(L, L->stacksize + n);
}


static CallInfo *growCI (lua_State *L) {
	if (L->size_ci > LUAI_MAXCALLS)  /* overflow while handling overflow? */
		luaD_throw(L, LUA_ERRERR);
	else {
		luaD_reallocCI(L, 2*L->size_ci);
		if (L->size_ci > LUAI_MAXCALLS)
			luaG_runerror(L, "stack overflow");
	}
	return ++L->ci;
}


void luaD_callhook (lua_State *L, int event, int line) {
	lua_Hook hook = L->hook;
	if (hook && L->allowhook) {
		ptrdiff_t top = savestack(L, L->top);
		ptrdiff_t ci_top = savestack(L, L->ci->top);
		lua_Debug ar;
		ar.event = event;
		ar.currentline = line;
		if (event == LUA_HOOKTAILRET)
			ar.i_ci = 0;  /* tail call; no debug information about it */
		else
			ar.i_ci = cast_int(L->ci - L->base_ci);
		luaD_checkstack(L, LUA_MINSTACK);  /* ensure minimum stack size */
		L->ci->top = L->top + LUA_MINSTACK;
		lua_assert(L->ci->top <= L->stack_last);
		L->allowhook = 0;  /* cannot call hooks inside a hook */
		lua_unlock(L);
		(*hook)(L, &ar);
		lua_lock(L);
		lua_assert(!L->allowhook);
		L->allowhook = 1;
		L->ci->top = restorestack(L, ci_top);
		L->top = restorestack(L, top);
	}
}


static StkId adjust_varargs (lua_State *L, Proto *p, int actual) {
	int i;
	int nfixargs = p->numparams;
	Table *htab = NULL;
	StkId base, fixed;
	for (; actual < nfixargs; ++actual)
		setnilvalue(L->top++);
#if defined(LUA_COMPAT_VARARG)
	if (p->is_vararg & VARARG_NEEDSARG) { /* compat. with old-style vararg? */
		int nvar = actual - nfixargs;  /* number of extra arguments */
		lua_assert(p->is_vararg & VARARG_HASARG);
		luaC_checkGC(L);
		htab = luaH_new(L, nvar, 1);  /* create `arg' table */
		for (i=0; i<nvar; i++)  /* put extra arguments into `arg' table */
			setobj2n(L, luaH_setnum(L, htab, i+1), L->top - nvar + i);
		/* store counter in field `n' */
		setnvalue(luaH_setstr(L, htab, luaS_newliteral(L, "n")), cast_num(nvar));
	}
#endif
	/* move fixed parameters to final position */
	fixed = L->top - actual;  /* first fixed argument */
	base = L->top;  /* final position of first argument */
	for (i=0; i<nfixargs; i++) {
		setobjs2s(L, L->top++, fixed+i);
		setnilvalue(fixed+i);
	}
	/* add `arg' parameter */
	if (htab) {
		sethvalue(L, L->top++, htab);
		lua_assert(iswhite(obj2gco(htab)));
	}
	return base;
}


static StkId tryfuncTM (lua_State *L, StkId func) {
	const TValue *tm = luaT_gettmbyobj(L, func, TM_CALL);
	StkId p;
	ptrdiff_t funcr = savestack(L, func);
	if (!ttisfunction(tm))
		luaG_typeerror(L, func, "call");
	/* Open a hole inside the stack at `func' */
	for (p = L->top; p > func; p--) setobjs2s(L, p, p-1);
	incr_top(L);
	func = restorestack(L, funcr);  /* previous call may change stack */
	setobj2s(L, func, tm);  /* tag method is the new function to be called */
	return func;
}



#define inc_ci(L) \
	((L->ci == L->end_ci) ? growCI(L) : \
	(condhardstacktests(luaD_reallocCI(L, L->size_ci)), ++L->ci))


int luaD_precall (lua_State *L, StkId func, int nresults) {
	LClosure *cl;
	ptrdiff_t funcr;
	if (!ttisfunction(func)) /* `func' is not a function? */
		func = tryfuncTM(L, func);  /* check the `function' tag method */
	funcr = savestack(L, func);
	cl = &clvalue(func)->l;
	L->ci->savedpc = L->savedpc;
	if (!cl->isC) {  /* Lua function? prepare its call */
		CallInfo *ci;
		StkId st, base;
		Proto *p = cl->p;
		luaD_checkstack(L, p->maxstacksize);
		func = restorestack(L, funcr);
		if (!p->is_vararg) {  /* no varargs? */
			base = func + 1;
			if (L->top > base + p->numparams)
				L->top = base + p->numparams;
		}
		else {  /* vararg function */
			int nargs = cast_int(L->top - func) - 1;
			base = adjust_varargs(L, p, nargs);
			func = restorestack(L, funcr);  /* previous call may change the stack */
		}
		ci = inc_ci(L);  /* now `enter' new function */
		ci->func = func;
		L->base = ci->base = base;
		ci->top = L->base + p->maxstacksize;
		lua_assert(ci->top <= L->stack_last);
		L->savedpc = p->code;  /* starting point */
		ci->tailcalls = 0;
		ci->nresults = nresults;
		for (st = L->top; st < ci->top; st++)
			setnilvalue(st);
		L->top = ci->top;
		if (L->hookmask & LUA_MASKCALL) {
			L->savedpc++;  /* hooks assume 'pc' is already incremented */
			luaD_callhook(L, LUA_HOOKCALL, -1);
			L->savedpc--;  /* correct 'pc' */
		}
		return PCRLUA;
	}
	else {  /* if is a C function, call it */
		CallInfo *ci;
		int n;
		luaD_checkstack(L, LUA_MINSTACK);  /* ensure minimum stack size */
		ci = inc_ci(L);  /* now `enter' new function */
		ci->func = restorestack(L, funcr);
		L->base = ci->base = ci->func + 1;
		ci->top = L->top + LUA_MINSTACK;
		lua_assert(ci->top <= L->stack_last);
		ci->nresults = nresults;
		if (L->hookmask & LUA_MASKCALL)
			luaD_callhook(L, LUA_HOOKCALL, -1);
		lua_unlock(L);
		n = (*curr_func(L)->c.f)(L);  /* do the actual call */
		lua_lock(L);
		if (n < 0)  /* yielding? */
			return PCRYIELD;
		else {
			luaD_poscall(L, L->top - n);
			return PCRC;
		}
	}
}


static StkId callrethooks (lua_State *L, StkId firstResult) {
	ptrdiff_t fr = savestack(L, firstResult);  /* next call may change stack */
	luaD_callhook(L, LUA_HOOKRET, -1);
	if (f_isLua(L->ci)) {  /* Lua function? */
		while (L->ci->tailcalls--)  /* call hook for eventual tail calls */
			luaD_callhook(L, LUA_HOOKTAILRET, -1);
	}
	return restorestack(L, fr);
}


int luaD_poscall (lua_State *L, StkId firstResult) {
	StkId res;
	int wanted, i;
	CallInfo *ci;
	if (L->hookmask & LUA_MASKRET)
		firstResult = callrethooks(L, firstResult);
	ci = L->ci--;
	res = ci->func;  /* res == final position of 1st result */
	wanted = ci->nresults;
	L->base = (ci - 1)->base;  /* restore base */
	L->savedpc = (ci - 1)->savedpc;  /* restore savedpc */
	/* move results to correct place */
	for (i = wanted; i != 0 && firstResult < L->top; i--)
		setobjs2s(L, res++, firstResult++);
	while (i-- > 0)
		setnilvalue(res++);
	L->top = res;
	return (wanted - LUA_MULTRET);  /* 0 iff wanted == LUA_MULTRET */
}


/*
** Call a function (C or Lua). The function to be called is at *func.
** The arguments are on the stack, right after the function.
** When returns, all the results are on the stack, starting at the original
** function position.
*/ 
void luaD_call (lua_State *L, StkId func, int nResults) {
	if (++L->nCcalls >= LUAI_MAXCCALLS) {
		if (L->nCcalls == LUAI_MAXCCALLS)
			luaG_runerror(L, "C stack overflow");
		else if (L->nCcalls >= (LUAI_MAXCCALLS + (LUAI_MAXCCALLS>>3)))
			luaD_throw(L, LUA_ERRERR);  /* error while handing stack error */
	}
	if (luaD_precall(L, func, nResults) == PCRLUA)  /* is a Lua function? */
		luaV_execute(L, 1);  /* call it */
	L->nCcalls--;
	luaC_checkGC(L);
}


static void resume (lua_State *L, void *ud) {
	StkId firstArg = cast(StkId, ud);
	CallInfo *ci = L->ci;
	if (L->status == 0) {  /* start coroutine? */
		lua_assert(ci == L->base_ci && firstArg > L->base);
		if (luaD_precall(L, firstArg - 1, LUA_MULTRET) != PCRLUA)
			return;
	}
	else {  /* resuming from previous yield */
		lua_assert(L->status == LUA_YIELD);
		L->status = 0;
		if (!f_isLua(ci)) {  /* `common' yield? */
			/* finish interrupted execution of `OP_CALL' */
			lua_assert(GET_OPCODE(*((ci-1)->savedpc - 1)) == OP_CALL ||
				GET_OPCODE(*((ci-1)->savedpc - 1)) == OP_TAILCALL);
			if (luaD_poscall(L, firstArg))  /* complete it... */
				L->top = L->ci->top;  /* and correct top if not multiple results */
		}
		else  /* yielded inside a hook: just continue its execution */
			L->base = L->ci->base;
	}
	luaV_execute(L, cast_int(L->ci - L->base_ci));
}


static int resume_error (lua_State *L, const char *msg) {
	L->top = L->ci->base;
	setsvalue2s(L, L->top, luaS_new(L, msg));
	incr_top(L);
	lua_unlock(L);
	return LUA_ERRRUN;
}


LUA_API int lua_resume (lua_State *L, int nargs) {
	int status;
	lua_lock(L);
	if (L->status != LUA_YIELD) {
		if (L->status != 0)
			return resume_error(L, "cannot resume dead coroutine");
		else if (L->ci != L->base_ci)
			return resume_error(L, "cannot resume non-suspended coroutine");
	}
	luai_userstateresume(L, nargs);
	lua_assert(L->errfunc == 0 && L->nCcalls == 0);
	status = luaD_rawrunprotected(L, resume, L->top - nargs);
	if (status != 0) {  /* error? */
		L->status = cast_byte(status);  /* mark thread as `dead' */
		luaD_seterrorobj(L, status, L->top);
		L->ci->top = L->top;
	}
	else
		status = L->status;
	lua_unlock(L);
	return status;
}


LUA_API int lua_yield (lua_State *L, int nresults) {
	luai_userstateyield(L, nresults);
	lua_lock(L);
	if (L->nCcalls > 0)
		luaG_runerror(L, "attempt to yield across metamethod/C-call boundary");
	L->base = L->top - nresults;  /* protect stack slots below */
	L->status = LUA_YIELD;
	lua_unlock(L);
	return -1;
}


int luaD_pcall (lua_State *L, Pfunc func, void *u,
				ptrdiff_t old_top, ptrdiff_t ef) {
					int status;
					unsigned short oldnCcalls = L->nCcalls;
					ptrdiff_t old_ci = saveci(L, L->ci);
					lu_byte old_allowhooks = L->allowhook;
					ptrdiff_t old_errfunc = L->errfunc;
					L->errfunc = ef;
					status = luaD_rawrunprotected(L, func, u);
					if (status != 0) {  /* an error occurred? */
						StkId oldtop = restorestack(L, old_top);
						luaF_close(L, oldtop);  /* close eventual pending closures */
						luaD_seterrorobj(L, status, oldtop);
						L->nCcalls = oldnCcalls;
						L->ci = restoreci(L, old_ci);
						L->base = L->ci->base;
						L->savedpc = L->ci->savedpc;
						L->allowhook = old_allowhooks;
						restore_stack_limit(L);
					}
					L->errfunc = old_errfunc;
					return status;
}



/*
** Execute a protected parser.
*/
struct SParser {  /* data to `f_parser' */
	ZIO *z;
	Mbuffer buff;  /* buffer to be used by the scanner */
	const char *name;
};

static void f_parser (lua_State *L, void *ud) {
	int i;
	Proto *tf;
	Closure *cl;
	struct SParser *p = cast(struct SParser *, ud);
	int c = luaZ_lookahead(p->z);
	luaC_checkGC(L);
	tf = ((c == LUA_SIGNATURE[0]) ? luaU_undump : luaY_parser)(L, p->z,
		&p->buff, p->name);
	cl = luaF_newLclosure(L, tf->nups, hvalue(gt(L)));
	cl->l.p = tf;
	for (i = 0; i < tf->nups; i++)  /* initialize eventual upvalues */
		cl->l.upvals[i] = luaF_newupval(L);
	setclvalue(L, L->top, cl);
	incr_top(L);
}


int luaD_protectedparser (lua_State *L, ZIO *z, const char *name) {
	struct SParser p;
	int status;
	p.z = z; p.name = name;
	luaZ_initbuffer(L, &p.buff);
	status = luaD_pcall(L, f_parser, &p, savestack(L, L->top), L->errfunc);
	luaZ_freebuffer(L, &p.buff);
	return status;
}


//-------------------------------------------------------------ltm.c--------------------------------------------------------
const char *const luaT_typenames[] = {
	"nil", "boolean", "userdata", "number",
	"string", "table", "function", "userdata", "thread",
	"proto", "upval"
};


void luaT_init (lua_State *L) {
	static const char *const luaT_eventname[] = {  /* ORDER TM */
		"__index", "__newindex",
		"__gc", "__mode", "__eq",
		"__add", "__sub", "__mul", "__div", "__mod",
		"__pow", "__unm", "__len", "__lt", "__le",
		"__concat", "__call"
	};
	int i;
	for (i=0; i<TM_N; i++) {
		G(L)->tmname[i] = luaS_new(L, luaT_eventname[i]);
		luaS_fix(G(L)->tmname[i]);  /* never collect these names */
	}
}


/*
** function to be used with macro "fasttm": optimized for absence of
** tag methods
*/
const TValue *luaT_gettm (Table *events, TMS event, TString *ename) {
	const TValue *tm = luaH_getstr(events, ename);
	lua_assert(event <= TM_EQ);
	if (ttisnil(tm)) {  /* no tag method? */
		events->flags |= cast_byte(1u<<event);  /* cache this fact */
		return NULL;
	}
	else return tm;
}


const TValue *luaT_gettmbyobj (lua_State *L, const TValue *o, TMS event) {
	Table *mt;
	switch (ttype(o)) {
	case LUA_TTABLE:
		mt = hvalue(o)->metatable;
		break;
	case LUA_TUSERDATA:
		mt = uvalue(o)->metatable;
		break;
	default:
		mt = G(L)->mt[ttype(o)];
	}
	return (mt ? luaH_getstr(mt, G(L)->tmname[event]) : luaO_nilobject);
}

//-------------------------------------------------------------lvm.c--------------------------------------------------------

/* limit for table tag-method chains (to avoid loops) */
#define MAXTAGLOOP	100

const TValue *luaV_tonumber (const TValue *obj, TValue *n) {
	lua_Number num;
	if (ttisnumber(obj)) return obj;
	if (ttisstring(obj) && luaO_str2d(svalue(obj), &num)) {
		setnvalue(n, num);
		return n;
	}
	else
		return NULL;
}


int luaV_tostring (lua_State *L, StkId obj) {
	if (!ttisnumber(obj))
		return 0;
	else {
		char s[LUAI_MAXNUMBER2STR];
		lua_Number n = nvalue(obj);
		lua_number2str(s, n);
		setsvalue2s(L, obj, luaS_new(L, s));
		return 1;
	}
}


static void traceexec (lua_State *L, const Instruction *pc) {
	lu_byte mask = L->hookmask;
	const Instruction *oldpc = L->savedpc;
	L->savedpc = pc;
	if (mask > LUA_MASKLINE) {  /* instruction-hook set? */
		if (L->hookcount == 0) {
			resethookcount(L);
			luaD_callhook(L, LUA_HOOKCOUNT, -1);
		}
	}
	if (mask & LUA_MASKLINE) {
		Proto *p = ci_func(L->ci)->l.p;
		int npc = pcRel(pc, p);
		int newline = getline(p, npc);
		/* call linehook when enter a new function, when jump back (loop),
		or when enter a new line */
		if (npc == 0 || pc <= oldpc || newline != getline(p, pcRel(oldpc, p)))
			luaD_callhook(L, LUA_HOOKLINE, newline);
	}
}


static void callTMres (lua_State *L, StkId res, const TValue *f, const TValue *p1, const TValue *p2) {
						   ptrdiff_t result = savestack(L, res);
						   setobj2s(L, L->top, f);  /* push function */
						   setobj2s(L, L->top+1, p1);  /* 1st argument */
						   setobj2s(L, L->top+2, p2);  /* 2nd argument */
						   luaD_checkstack(L, 3);
						   L->top += 3;
						   luaD_call(L, L->top - 3, 1);
						   res = restorestack(L, result);
						   L->top--;
						   setobjs2s(L, res, L->top);
}



static void callTM (lua_State *L, const TValue *f, const TValue *p1, const TValue *p2, const TValue *p3) {
						setobj2s(L, L->top, f);  /* push function */
						setobj2s(L, L->top+1, p1);  /* 1st argument */
						setobj2s(L, L->top+2, p2);  /* 2nd argument */
						setobj2s(L, L->top+3, p3);  /* 3th argument */
						luaD_checkstack(L, 4);
						L->top += 4;
						luaD_call(L, L->top - 4, 0);
}


void luaV_gettable (lua_State *L, const TValue *t, TValue *key, StkId val) {
	int loop;
	for (loop = 0; loop < MAXTAGLOOP; loop++) {
		const TValue *tm;
		if (ttistable(t)) {  /* `t' is a table? */
			Table *h = hvalue(t);
			const TValue *res = luaH_get(h, key); /* do a primitive get */
			if (!ttisnil(res) ||  /* result is no nil? */
				(tm = fasttm(L, h->metatable, TM_INDEX)) == NULL) { /* or no TM? */
					setobj2s(L, val, res);
					return;
			}
			/* else will try the tag method */
		}
		else if (ttisnil(tm = luaT_gettmbyobj(L, t, TM_INDEX)))
			luaG_typeerror(L, t, "index");
		if (ttisfunction(tm)) {
			callTMres(L, val, tm, t, key);
			return;
		}
		t = tm;  /* else repeat with `tm' */ 
	}
	luaG_runerror(L, "loop in gettable");
}


void luaV_settable (lua_State *L, const TValue *t, TValue *key, StkId val) {
	int loop;
	for (loop = 0; loop < MAXTAGLOOP; loop++) {
		const TValue *tm;
		if (ttistable(t)) {  /* `t' is a table? */
			Table *h = hvalue(t);
			TValue *oldval = luaH_set(L, h, key); /* do a primitive set */
			if (!ttisnil(oldval) ||  /* result is no nil? */
				(tm = fasttm(L, h->metatable, TM_NEWINDEX)) == NULL) { /* or no TM? */
					setobj2t(L, oldval, val);
					luaC_barriert(L, h, val);
					return;
			}
			/* else will try the tag method */
		}
		else if (ttisnil(tm = luaT_gettmbyobj(L, t, TM_NEWINDEX)))
			luaG_typeerror(L, t, "index");
		if (ttisfunction(tm)) {
			callTM(L, tm, t, key, val);
			return;
		}
		t = tm;  /* else repeat with `tm' */ 
	}
	luaG_runerror(L, "loop in settable");
}


static int call_binTM (lua_State *L, const TValue *p1, const TValue *p2, StkId res, TMS event) {
						   const TValue *tm = luaT_gettmbyobj(L, p1, event);  /* try first operand */
						   if (ttisnil(tm))
							   tm = luaT_gettmbyobj(L, p2, event);  /* try second operand */
						   if (ttisnil(tm)) return 0;
						   callTMres(L, res, tm, p1, p2);
						   return 1;
}


static const TValue *get_compTM (lua_State *L, Table *mt1, Table *mt2, TMS event) {
									 const TValue *tm1 = fasttm(L, mt1, event);
									 const TValue *tm2;
									 if (tm1 == NULL) return NULL;  /* no metamethod */
									 if (mt1 == mt2) return tm1;  /* same metatables => same metamethods */
									 tm2 = fasttm(L, mt2, event);
									 if (tm2 == NULL) return NULL;  /* no metamethod */
									 if (luaO_rawequalObj(tm1, tm2))  /* same metamethods? */
										 return tm1;
									 return NULL;
}


static int call_orderTM (lua_State *L, const TValue *p1, const TValue *p2, TMS event) {
							 const TValue *tm1 = luaT_gettmbyobj(L, p1, event);
							 const TValue *tm2;
							 if (ttisnil(tm1)) return -1;  /* no metamethod? */
							 tm2 = luaT_gettmbyobj(L, p2, event);
							 if (!luaO_rawequalObj(tm1, tm2))  /* different metamethods? */
								 return -1;
							 callTMres(L, L->top, tm1, p1, p2);
							 return !l_isfalse(L->top);
}


static int l_strcmp (const TString *ls, const TString *rs) {
	const char *l = getstr(ls);
	size_t ll = ls->tsv.len;
	const char *r = getstr(rs);
	size_t lr = rs->tsv.len;
	for (;;) {
		int temp = strcoll(l, r);
		if (temp != 0) return temp;
		else {  /* strings are equal up to a `\0' */
			size_t len = strlen(l);  /* index of first `\0' in both strings */
			if (len == lr)  /* r is finished? */
				return (len == ll) ? 0 : 1;
			else if (len == ll)  /* l is finished? */
				return -1;  /* l is smaller than r (because r is not finished) */
			/* both strings longer than `len'; go on comparing (after the `\0') */
			len++;
			l += len; ll -= len; r += len; lr -= len;
		}
	}
}


int luaV_lessthan (lua_State *L, const TValue *l, const TValue *r) {
	int res;
	if (ttype(l) != ttype(r))
		return luaG_ordererror(L, l, r);
	else if (ttisnumber(l))
		return luai_numlt(nvalue(l), nvalue(r));
	else if (ttisstring(l))
		return l_strcmp(rawtsvalue(l), rawtsvalue(r)) < 0;
	else if ((res = call_orderTM(L, l, r, TM_LT)) != -1)
		return res;
	return luaG_ordererror(L, l, r);
}


static int lessequal (lua_State *L, const TValue *l, const TValue *r) {
	int res;
	if (ttype(l) != ttype(r))
		return luaG_ordererror(L, l, r);
	else if (ttisnumber(l))
		return luai_numle(nvalue(l), nvalue(r));
	else if (ttisstring(l))
		return l_strcmp(rawtsvalue(l), rawtsvalue(r)) <= 0;
	else if ((res = call_orderTM(L, l, r, TM_LE)) != -1)  /* first try `le' */
		return res;
	else if ((res = call_orderTM(L, r, l, TM_LT)) != -1)  /* else try `lt' */
		return !res;
	return luaG_ordererror(L, l, r);
}


int luaV_equalval (lua_State *L, const TValue *t1, const TValue *t2) {
	const TValue *tm;
	lua_assert(ttype(t1) == ttype(t2));
	switch (ttype(t1)) {
	case LUA_TNIL: return 1;
	case LUA_TNUMBER: return luai_numeq(nvalue(t1), nvalue(t2));
	case LUA_TBOOLEAN: return bvalue(t1) == bvalue(t2);  /* true must be 1 !! */
	case LUA_TLIGHTUSERDATA: return pvalue(t1) == pvalue(t2);
	case LUA_TUSERDATA: {
		if (uvalue(t1) == uvalue(t2)) return 1;
		tm = get_compTM(L, uvalue(t1)->metatable, uvalue(t2)->metatable,
			TM_EQ);
		break;  /* will try TM */
						}
	case LUA_TTABLE: {
		if (hvalue(t1) == hvalue(t2)) return 1;
		tm = get_compTM(L, hvalue(t1)->metatable, hvalue(t2)->metatable, TM_EQ);
		break;  /* will try TM */
					 }
	default: return gcvalue(t1) == gcvalue(t2);
	}
	if (tm == NULL) return 0;  /* no TM? */
	callTMres(L, L->top, tm, t1, t2);  /* call TM */
	return !l_isfalse(L->top);
}


void luaV_concat (lua_State *L, int total, int last) {
	do {
		StkId top = L->base + last + 1;
		int n = 2;  /* number of elements handled in this pass (at least 2) */
		if (!(ttisstring(top-2) || ttisnumber(top-2)) || !tostring(L, top-1)) {
			if (!call_binTM(L, top-2, top-1, top-2, TM_CONCAT))
				luaG_concaterror(L, top-2, top-1);
		} else if (tsvalue(top-1)->len == 0)  /* second op is empty? */
			(void)tostring(L, top - 2);  /* result is first op (as string) */
		else {
			/* at least two string values; get as many as possible */
			size_t tl = tsvalue(top-1)->len;
			char *buffer;
			int i;
			/* collect total length */
			for (n = 1; n < total && tostring(L, top-n-1); n++) {
				size_t l = tsvalue(top-n-1)->len;
				if (l >= MAX_SIZET - tl) luaG_runerror(L, "string length overflow");
				tl += l;
			}
			buffer = luaZ_openspace(L, &G(L)->buff, tl);
			tl = 0;
			for (i=n; i>0; i--) {  /* concat all strings */
				size_t l = tsvalue(top-i)->len;
				memcpy(buffer+tl, svalue(top-i), l);
				tl += l;
			}
			setsvalue2s(L, top-n, luaS_newlstr(L, buffer, tl));
		}
		total -= n-1;  /* got `n' strings to create 1 new */
		last -= n-1;
	} while (total > 1);  /* repeat until only 1 result left */
}


static void Arith (lua_State *L, StkId ra, const TValue *rb, const TValue *rc, TMS op) {
					   TValue tempb, tempc;
					   const TValue *b, *c;
					   if ((b = luaV_tonumber(rb, &tempb)) != NULL &&
						   (c = luaV_tonumber(rc, &tempc)) != NULL) {
							   lua_Number nb = nvalue(b), nc = nvalue(c);
							   switch (op) {
	  case TM_ADD: setnvalue(ra, luai_numadd(nb, nc)); break;
	  case TM_SUB: setnvalue(ra, luai_numsub(nb, nc)); break;
	  case TM_MUL: setnvalue(ra, luai_nummul(nb, nc)); break;
	  case TM_DIV: setnvalue(ra, luai_numdiv(nb, nc)); break;
	  case TM_MOD: setnvalue(ra, luai_nummod(nb, nc)); break;
	  case TM_POW: setnvalue(ra, luai_numpow(nb, nc)); break;
	  case TM_UNM: setnvalue(ra, luai_numunm(nb)); break;
	  default: lua_assert(0); break;
							   }
					   }
					   else if (!call_binTM(L, rb, rc, ra, op))
						   luaG_aritherror(L, rb, rc);
}



/*
** some macros for common tasks in `luaV_execute'
*/

#define runtime_check(L, c)	{ if (!(c)) break; }

#define RA(i)	(base+GETARG_A(i))
/* to be used after possible stack reallocation */
#define RB(i)	check_exp(getBMode(GET_OPCODE(i)) == OpArgR, base+GETARG_B(i))
#define RC(i)	check_exp(getCMode(GET_OPCODE(i)) == OpArgR, base+GETARG_C(i))
#define RKB(i)	check_exp(getBMode(GET_OPCODE(i)) == OpArgK, \
	ISK(GETARG_B(i)) ? k+INDEXK(GETARG_B(i)) : base+GETARG_B(i))
#define RKC(i)	check_exp(getCMode(GET_OPCODE(i)) == OpArgK, \
	ISK(GETARG_C(i)) ? k+INDEXK(GETARG_C(i)) : base+GETARG_C(i))
#define KBx(i)	check_exp(getBMode(GET_OPCODE(i)) == OpArgK, k+GETARG_Bx(i))


#define dojump(L,pc,i)	{(pc) += (i); luai_threadyield(L);}


#define Protect(x)	{ L->savedpc = pc; {x;}; base = L->base; }


#define arith_op(op,tm) { \
	TValue *rb = RKB(i); \
	TValue *rc = RKC(i); \
	if (ttisnumber(rb) && ttisnumber(rc)) { \
	lua_Number nb = nvalue(rb), nc = nvalue(rc); \
	setnvalue(ra, op(nb, nc)); \
	} \
		else \
		Protect(Arith(L, ra, rb, rc, tm)); \
}

void luaV_execute (lua_State *L, int nexeccalls) {
	LClosure *cl;
	StkId base;
	TValue *k;
	const Instruction *pc;
reentry:  /* entry point */
	lua_assert(isLua(L->ci));
	pc = L->savedpc;
	cl = &clvalue(L->ci->func)->l;
	base = L->base;
	k = cl->p->k;
	/* main loop of interpreter */
	for (;;) {
		const Instruction i = *pc++;
		StkId ra;
		if ((L->hookmask & (LUA_MASKLINE | LUA_MASKCOUNT)) &&
			(--L->hookcount == 0 || L->hookmask & LUA_MASKLINE)) {
				traceexec(L, pc);
				if (L->status == LUA_YIELD) {  /* did hook yield? */
					L->savedpc = pc - 1;
					return;
				}
				base = L->base;
		}
		/* warning!! several calls may realloc the stack and invalidate `ra' */
		ra = RA(i);
		lua_assert(base == L->base && L->base == L->ci->base);
		lua_assert(base <= L->top && L->top <= L->stack + L->stacksize);
		lua_assert(L->top == L->ci->top || luaG_checkopenop(i));
		switch (GET_OPCODE(i)) {
	  case OP_MOVE: {
		  setobjs2s(L, ra, RB(i));
		  continue;
					}
	  case OP_LOADK: {
		  setobj2s(L, ra, KBx(i));
		  continue;
					 }
	  case OP_LOADBOOL: {
		  setbvalue(ra, GETARG_B(i));
		  if (GETARG_C(i)) pc++;  /* skip next instruction (if C) */
		  continue;
						}
	  case OP_LOADNIL: {
		  TValue *rb = RB(i);
		  do {
			  setnilvalue(rb--);
		  } while (rb >= ra);
		  continue;
					   }
	  case OP_GETUPVAL: {
		  int b = GETARG_B(i);
		  setobj2s(L, ra, cl->upvals[b]->v);
		  continue;
						}
	  case OP_GETGLOBAL: {
		  TValue g;
		  TValue *rb = KBx(i);
		  sethvalue(L, &g, cl->env);
		  lua_assert(ttisstring(rb));
		  Protect(luaV_gettable(L, &g, rb, ra));
		  continue;
						 }
	  case OP_GETTABLE: {
		  Protect(luaV_gettable(L, RB(i), RKC(i), ra));
		  continue;
						}
	  case OP_SETGLOBAL: {
		  TValue g;
		  sethvalue(L, &g, cl->env);
		  lua_assert(ttisstring(KBx(i)));
		  Protect(luaV_settable(L, &g, KBx(i), ra));
		  continue;
						 }
	  case OP_SETUPVAL: {
		  UpVal *uv = cl->upvals[GETARG_B(i)];
		  setobj(L, uv->v, ra);
		  luaC_barrier(L, uv, ra);
		  continue;
						}
	  case OP_SETTABLE: {
		  Protect(luaV_settable(L, ra, RKB(i), RKC(i)));
		  continue;
						}
	  case OP_NEWTABLE: {
		  int b = GETARG_B(i);
		  int c = GETARG_C(i);
		  sethvalue(L, ra, luaH_new(L, luaO_fb2int(b), luaO_fb2int(c)));
		  Protect(luaC_checkGC(L));
		  continue;
						}
	  case OP_SELF: {
		  StkId rb = RB(i);
		  setobjs2s(L, ra+1, rb);
		  Protect(luaV_gettable(L, rb, RKC(i), ra));
		  continue;
					}
	  case OP_ADD: {
		  arith_op(luai_numadd, TM_ADD);
		  continue;
				   }
	  case OP_SUB: {
		  arith_op(luai_numsub, TM_SUB);
		  continue;
				   }
	  case OP_MUL: {
		  arith_op(luai_nummul, TM_MUL);
		  continue;
				   }
	  case OP_DIV: {
		  arith_op(luai_numdiv, TM_DIV);
		  continue;
				   }
	  case OP_MOD: {
		  arith_op(luai_nummod, TM_MOD);
		  continue;
				   }
	  case OP_POW: {
		  arith_op(luai_numpow, TM_POW);
		  continue;
				   }
	  case OP_UNM: {
		  TValue *rb = RB(i);
		  if (ttisnumber(rb)) {
			  lua_Number nb = nvalue(rb);
			  setnvalue(ra, luai_numunm(nb));
		  }
		  else {
			  Protect(Arith(L, ra, rb, rb, TM_UNM));
		  }
		  continue;
				   }
	  case OP_NOT: {
		  int res = l_isfalse(RB(i));  /* next assignment may change this value */
		  setbvalue(ra, res);
		  continue;
				   }
	  case OP_LEN: {
		  const TValue *rb = RB(i);
		  switch (ttype(rb)) {
	  case LUA_TTABLE: {
		  setnvalue(ra, cast_num(luaH_getn(hvalue(rb))));
		  break;
					   }
	  case LUA_TSTRING: {
		  setnvalue(ra, cast_num(tsvalue(rb)->len));
		  break;
						}
	  default: {  /* try metamethod */
		  Protect(
			  if (!call_binTM(L, rb, luaO_nilobject, ra, TM_LEN))
				  luaG_typeerror(L, rb, "get length of");
		  )
			   }
		  }
		  continue;
				   }
	  case OP_CONCAT: {
		  int b = GETARG_B(i);
		  int c = GETARG_C(i);
		  Protect(luaV_concat(L, c-b+1, c); luaC_checkGC(L));
		  setobjs2s(L, RA(i), base+b);
		  continue;
					  }
	  case OP_JMP: {
		  dojump(L, pc, GETARG_sBx(i));
		  continue;
				   }
	  case OP_EQ: {
		  TValue *rb = RKB(i);
		  TValue *rc = RKC(i);
		  Protect(
			  if (equalobj(L, rb, rc) == GETARG_A(i))
				  dojump(L, pc, GETARG_sBx(*pc));
		  )
			  pc++;
		  continue;
				  }
	  case OP_LT: {
		  Protect(
			  if (luaV_lessthan(L, RKB(i), RKC(i)) == GETARG_A(i))
				  dojump(L, pc, GETARG_sBx(*pc));
		  )
			  pc++;
		  continue;
				  }
	  case OP_LE: {
		  Protect(
			  if (lessequal(L, RKB(i), RKC(i)) == GETARG_A(i))
				  dojump(L, pc, GETARG_sBx(*pc));
		  )
			  pc++;
		  continue;
				  }
	  case OP_TEST: {
		  if (l_isfalse(ra) != GETARG_C(i))
			  dojump(L, pc, GETARG_sBx(*pc));
		  pc++;
		  continue;
					}
	  case OP_TESTSET: {
		  TValue *rb = RB(i);
		  if (l_isfalse(rb) != GETARG_C(i)) {
			  setobjs2s(L, ra, rb);
			  dojump(L, pc, GETARG_sBx(*pc));
		  }
		  pc++;
		  continue;
					   }
	  case OP_CALL: {
		  int b = GETARG_B(i);
		  int nresults = GETARG_C(i) - 1;
		  if (b != 0) L->top = ra+b;  /* else previous instruction set top */
		  L->savedpc = pc;
		  switch (luaD_precall(L, ra, nresults)) {
	  case PCRLUA: {
		  nexeccalls++;
		  goto reentry;  /* restart luaV_execute over new Lua function */
				   }
	  case PCRC: {
		  /* it was a C function (`precall' called it); adjust results */
		  if (nresults >= 0) L->top = L->ci->top;
		  base = L->base;
		  continue;
				 }
	  default: {
		  return;  /* yield */
			   }
		  }
					}
	  case OP_TAILCALL: {
		  int b = GETARG_B(i);
		  if (b != 0) L->top = ra+b;  /* else previous instruction set top */
		  L->savedpc = pc;
		  lua_assert(GETARG_C(i) - 1 == LUA_MULTRET);
		  switch (luaD_precall(L, ra, LUA_MULTRET)) {
	  case PCRLUA: {
		  /* tail call: put new frame in place of previous one */
		  CallInfo *ci = L->ci - 1;  /* previous frame */
		  int aux;
		  StkId func = ci->func;
		  StkId pfunc = (ci+1)->func;  /* previous function index */
		  if (L->openupval) luaF_close(L, ci->base);
		  L->base = ci->base = ci->func + ((ci+1)->base - pfunc);
		  for (aux = 0; pfunc+aux < L->top; aux++)  /* move frame down */
			  setobjs2s(L, func+aux, pfunc+aux);
		  ci->top = L->top = func+aux;  /* correct top */
		  lua_assert(L->top == L->base + clvalue(func)->l.p->maxstacksize);
		  ci->savedpc = L->savedpc;
		  ci->tailcalls++;  /* one more call lost */
		  L->ci--;  /* remove new frame */
		  goto reentry;
				   }
	  case PCRC: {  /* it was a C function (`precall' called it) */
		  base = L->base;
		  continue;
				 }
	  default: {
		  return;  /* yield */
			   }
		  }
						}
	  case OP_RETURN: {
		  int b = GETARG_B(i);
		  if (b != 0) L->top = ra+b-1;
		  if (L->openupval) luaF_close(L, base);
		  L->savedpc = pc;
		  b = luaD_poscall(L, ra);
		  if (--nexeccalls == 0)  /* was previous function running `here'? */
			  return;  /* no: return */
		  else {  /* yes: continue its execution */
			  if (b) L->top = L->ci->top;
			  lua_assert(isLua(L->ci));
			  lua_assert(GET_OPCODE(*((L->ci)->savedpc - 1)) == OP_CALL);
			  goto reentry;
		  }
					  }
	  case OP_FORLOOP: {
		  lua_Number step = nvalue(ra+2);
		  lua_Number idx = luai_numadd(nvalue(ra), step); /* increment index */
		  lua_Number limit = nvalue(ra+1);
		  if (luai_numlt(0, step) ? luai_numle(idx, limit)
			  : luai_numle(limit, idx)) {
				  dojump(L, pc, GETARG_sBx(i));  /* jump back */
				  setnvalue(ra, idx);  /* update internal index... */
				  setnvalue(ra+3, idx);  /* ...and external index */
		  }
		  continue;
					   }
	  case OP_FORPREP: {
		  const TValue *init = ra;
		  const TValue *plimit = ra+1;
		  const TValue *pstep = ra+2;
		  L->savedpc = pc;  /* next steps may throw errors */
		  if (!tonumber(init, ra))
			  luaG_runerror(L, LUA_QL("for") " initial value must be a number");
		  else if (!tonumber(plimit, ra+1))
			  luaG_runerror(L, LUA_QL("for") " limit must be a number");
		  else if (!tonumber(pstep, ra+2))
			  luaG_runerror(L, LUA_QL("for") " step must be a number");
		  setnvalue(ra, luai_numsub(nvalue(ra), nvalue(pstep)));
		  dojump(L, pc, GETARG_sBx(i));
		  continue;
					   }
	  case OP_TFORLOOP: {
		  StkId cb = ra + 3;  /* call base */
		  setobjs2s(L, cb+2, ra+2);
		  setobjs2s(L, cb+1, ra+1);
		  setobjs2s(L, cb, ra);
		  L->top = cb+3;  /* func. + 2 args (state and index) */
		  Protect(luaD_call(L, cb, GETARG_C(i)));
		  L->top = L->ci->top;
		  cb = RA(i) + 3;  /* previous call may change the stack */
		  if (!ttisnil(cb)) {  /* continue loop? */
			  setobjs2s(L, cb-1, cb);  /* save control variable */
			  dojump(L, pc, GETARG_sBx(*pc));  /* jump back */
		  }
		  pc++;
		  continue;
						}
	  case OP_SETLIST: {
		  int n = GETARG_B(i);
		  int c = GETARG_C(i);
		  int last;
		  Table *h;
		  if (n == 0) {
			  n = cast_int(L->top - ra) - 1;
			  L->top = L->ci->top;
		  }
		  if (c == 0) c = cast_int(*pc++);
		  runtime_check(L, ttistable(ra));
		  h = hvalue(ra);
		  last = ((c-1)*LFIELDS_PER_FLUSH) + n;
		  if (last > h->sizearray)  /* needs more space? */
			  luaH_resizearray(L, h, last);  /* pre-alloc it at once */
		  for (; n > 0; n--) {
			  TValue *val = ra+n;
			  setobj2t(L, luaH_setnum(L, h, last--), val);
			  luaC_barriert(L, h, val);
		  }
		  continue;
					   }
	  case OP_CLOSE: {
		  luaF_close(L, ra);
		  continue;
					 }
	  case OP_CLOSURE: {
		  Proto *p;
		  Closure *ncl;
		  int nup, j;
		  p = cl->p->p[GETARG_Bx(i)];
		  nup = p->nups;
		  ncl = luaF_newLclosure(L, nup, cl->env);
		  ncl->l.p = p;
		  for (j=0; j<nup; j++, pc++) {
			  if (GET_OPCODE(*pc) == OP_GETUPVAL)
				  ncl->l.upvals[j] = cl->upvals[GETARG_B(*pc)];
			  else {
				  lua_assert(GET_OPCODE(*pc) == OP_MOVE);
				  ncl->l.upvals[j] = luaF_findupval(L, base + GETARG_B(*pc));
			  }
		  }
		  setclvalue(L, ra, ncl);
		  Protect(luaC_checkGC(L));
		  continue;
					   }
	  case OP_VARARG: {
		  int b = GETARG_B(i) - 1;
		  int j;
		  CallInfo *ci = L->ci;
		  int n = cast_int(ci->base - ci->func) - cl->p->numparams - 1;
		  if (b == LUA_MULTRET) {
			  Protect(luaD_checkstack(L, n));
			  ra = RA(i);  /* previous call may change the stack */
			  b = n;
			  L->top = ra + n;
		  }
		  for (j = 0; j < b; j++) {
			  if (j < n) {
				  setobjs2s(L, ra + j, ci->base - n + j);
			  }
			  else {
				  setnilvalue(ra + j);
			  }
		  }
		  continue;
					  }
		}
	}
}


//-------------------------------------------------------------lobject.c----------------------------------------------------
const TValue luaO_nilobject_ = {{NULL}, LUA_TNIL};


/*
** converts an integer to a "floating point byte", represented as
** (eeeeexxx), where the real value is (1xxx) * 2^(eeeee - 1) if
** eeeee != 0 and (xxx) otherwise.
*/
int luaO_int2fb (unsigned int x) {
	int e = 0;  /* expoent */
	while (x >= 16) {
		x = (x+1) >> 1;
		e++;
	}
	if (x < 8) return x;
	else return ((e+1) << 3) | (cast_int(x) - 8);
}


/* converts back */
int luaO_fb2int (int x) {
	int e = (x >> 3) & 31;
	if (e == 0) return x;
	else return ((x & 7)+8) << (e - 1);
}


int luaO_log2 (unsigned int x) {
	static const lu_byte log_2[256] = {
		0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8
	};
	int l = -1;
	while (x >= 256) { l += 8; x >>= 8; }
	return l + log_2[x];

}


int luaO_rawequalObj (const TValue *t1, const TValue *t2) {
	if (ttype(t1) != ttype(t2)) return 0;
	else switch (ttype(t1)) {
	case LUA_TNIL:
		return 1;
	case LUA_TNUMBER:
		return luai_numeq(nvalue(t1), nvalue(t2));
	case LUA_TBOOLEAN:
		return bvalue(t1) == bvalue(t2);  /* boolean true must be 1 !! */
	case LUA_TLIGHTUSERDATA:
		return pvalue(t1) == pvalue(t2);
	default:
		lua_assert(iscollectable(t1));
		return gcvalue(t1) == gcvalue(t2);
	}
}


int luaO_str2d (const char *s, lua_Number *result) {
	char *endptr;
	*result = lua_str2number(s, &endptr);
	if (endptr == s) return 0;  /* conversion failed */
	if (*endptr == 'x' || *endptr == 'X')  /* maybe an hexadecimal constant? */
		*result = cast_num(strtoul(s, &endptr, 16));
	if (*endptr == '\0') return 1;  /* most common case */
	while (isspace(cast(unsigned char, *endptr))) endptr++;
	if (*endptr != '\0') return 0;  /* invalid trailing characters? */
	return 1;
}



static void pushstr (lua_State *L, const char *str) {
	setsvalue2s(L, L->top, luaS_new(L, str));
	incr_top(L);
}


/* this function handles only `%d', `%c', %f, %p, and `%s' formats */
const char *luaO_pushvfstring (lua_State *L, const char *fmt, va_list argp) {
	int n = 1;
	pushstr(L, "");
	for (;;) {
		const char *e = strchr(fmt, '%');
		if (e == NULL) break;
		setsvalue2s(L, L->top, luaS_newlstr(L, fmt, e-fmt));
		incr_top(L);
		switch (*(e+1)) {
	  case 's': {
		  const char *s = va_arg(argp, char *);
		  if (s == NULL) s = "(null)";
		  pushstr(L, s);
		  break;
				}
	  case 'c': {
		  char buff[2];
		  buff[0] = cast(char, va_arg(argp, int));
		  buff[1] = '\0';
		  pushstr(L, buff);
		  break;
				}
	  case 'd': {
		  setnvalue(L->top, cast_num(va_arg(argp, int)));
		  incr_top(L);
		  break;
				}
	  case 'f': {
		  setnvalue(L->top, cast_num(va_arg(argp, l_uacNumber)));
		  incr_top(L);
		  break;
				}
	  case 'p': {
		  char buff[4*sizeof(void *) + 8]; /* should be enough space for a `%p' */
		  sprintf(buff, "%p", va_arg(argp, void *));
		  pushstr(L, buff);
		  break;
				}
	  case '%': {
		  pushstr(L, "%");
		  break;
				}
	  default: {
		  char buff[3];
		  buff[0] = '%';
		  buff[1] = *(e+1);
		  buff[2] = '\0';
		  pushstr(L, buff);
		  break;
			   }
		}
		n += 2;
		fmt = e+2;
	}
	pushstr(L, fmt);
	luaV_concat(L, n+1, cast_int(L->top - L->base) - 1);
	L->top -= n;
	return svalue(L->top - 1);
}


const char *luaO_pushfstring (lua_State *L, const char *fmt, ...) {
	const char *msg;
	va_list argp;
	va_start(argp, fmt);
	msg = luaO_pushvfstring(L, fmt, argp);
	va_end(argp);
	return msg;
}


void luaO_chunkid (char *out, const char *source, size_t bufflen) {
	if (*source == '=') {
		strncpy(out, source+1, bufflen);  /* remove first char */
		out[bufflen-1] = '\0';  /* ensures null termination */
	}
	else {  /* out = "source", or "...source" */
		if (*source == '@') {
			size_t l;
			source++;  /* skip the `@' */
			bufflen -= sizeof(" '...' ");
			l = strlen(source);
			strcpy(out, "");
			if (l > bufflen) {
				source += (l-bufflen);  /* get last part of file name */
				strcat(out, "...");
			}
			strcat(out, source);
		}
		else {  /* out = [string "string"] */
			size_t len = strcspn(source, "\n\r");  /* stop at first newline */
			bufflen -= sizeof(" [string \"...\"] ");
			if (len > bufflen) len = bufflen;
			strcpy(out, "[string \"");
			if (source[len] != '\0') {  /* must truncate? */
				strncat(out, source, len);
				strcat(out, "...");
			}
			else
				strcat(out, source);
			strcat(out, "\"]");
		}
	}
}

//-------------------------------------------------------------ltable.c-----------------------------------------------------
//From lua.h-------------------------------
/*
** basic stack manipulation
*/
LUA_API int   (lua_gettop) (lua_State *L);
LUA_API void  (lua_settop) (lua_State *L, int idx);
LUA_API void  (lua_pushvalue) (lua_State *L, int idx);
LUA_API void  (lua_remove) (lua_State *L, int idx);
LUA_API void  (lua_insert) (lua_State *L, int idx);
LUA_API void  (lua_replace) (lua_State *L, int idx);
LUA_API int   (lua_checkstack) (lua_State *L, int sz);

LUA_API void  (lua_xmove) (lua_State *from, lua_State *to, int n);


/*
** access functions (stack -> C)
*/
LUA_API int             (lua_isnumber) (lua_State *L, int idx);
LUA_API int             (lua_isstring) (lua_State *L, int idx);
LUA_API int             (lua_iscfunction) (lua_State *L, int idx);
LUA_API int             (lua_isuserdata) (lua_State *L, int idx);
LUA_API int             (lua_type) (lua_State *L, int idx);
LUA_API const char     *(lua_typename) (lua_State *L, int tp);

LUA_API int            (lua_equal) (lua_State *L, int idx1, int idx2);
LUA_API int            (lua_rawequal) (lua_State *L, int idx1, int idx2);
LUA_API int            (lua_lessthan) (lua_State *L, int idx1, int idx2);

LUA_API lua_Number      (lua_tonumber) (lua_State *L, int idx);
LUA_API lua_Integer     (lua_tointeger) (lua_State *L, int idx);
LUA_API int             (lua_toboolean) (lua_State *L, int idx);
LUA_API const char     *(lua_tolstring) (lua_State *L, int idx, size_t *len);
LUA_API size_t          (lua_objlen) (lua_State *L, int idx);
LUA_API lua_CFunction   (lua_tocfunction) (lua_State *L, int idx);
LUA_API void	       *(lua_touserdata) (lua_State *L, int idx);
LUA_API lua_State      *(lua_tothread) (lua_State *L, int idx);
LUA_API const void     *(lua_topointer) (lua_State *L, int idx);

/*
** push functions (C -> stack)
*/
LUA_API void  (lua_pushnil) (lua_State *L);
LUA_API void  (lua_pushnumber) (lua_State *L, lua_Number n);
LUA_API void  (lua_pushinteger) (lua_State *L, lua_Integer n);
LUA_API void  (lua_pushlstring) (lua_State *L, const char *s, size_t l);
LUA_API void  (lua_pushstring) (lua_State *L, const char *s);
LUA_API const char *(lua_pushvfstring) (lua_State *L, const char *fmt,
										va_list argp);
LUA_API const char *(lua_pushfstring) (lua_State *L, const char *fmt, ...);
LUA_API void  (lua_pushcclosure) (lua_State *L, lua_CFunction fn, int n);
LUA_API void  (lua_pushboolean) (lua_State *L, int b);
LUA_API void  (lua_pushlightuserdata) (lua_State *L, void *p);
LUA_API int   (lua_pushthread) (lua_State *L);


/*
** get functions (Lua -> stack)
*/
LUA_API void  (lua_gettable) (lua_State *L, int idx);
LUA_API void  (lua_getfield) (lua_State *L, int idx, const char *k);
LUA_API void  (lua_rawget) (lua_State *L, int idx);
LUA_API void  (lua_rawgeti) (lua_State *L, int idx, int n);
LUA_API void  (lua_createtable) (lua_State *L, int narr, int nrec);
LUA_API void *(lua_newuserdata) (lua_State *L, size_t sz);
LUA_API int   (lua_getmetatable) (lua_State *L, int objindex);
LUA_API void  (lua_getfenv) (lua_State *L, int idx);


/*
** set functions (stack -> Lua)
*/
LUA_API void  (lua_settable) (lua_State *L, int idx);
LUA_API void  (lua_setfield) (lua_State *L, int idx, const char *k);
LUA_API void  (lua_rawset) (lua_State *L, int idx);
LUA_API void  (lua_rawseti) (lua_State *L, int idx, int n);
LUA_API int   (lua_setmetatable) (lua_State *L, int objindex);
LUA_API int   (lua_setfenv) (lua_State *L, int idx);


/*
** `load' and `call' functions (load and run Lua code)
*/
LUA_API void  (lua_call) (lua_State *L, int nargs, int nresults);
LUA_API int   (lua_pcall) (lua_State *L, int nargs, int nresults, int errfunc);
LUA_API int   (lua_cpcall) (lua_State *L, lua_CFunction func, void *ud);
LUA_API int   (lua_load) (lua_State *L, lua_Reader reader, void *dt,
						  const char *chunkname);

LUA_API int (lua_dump) (lua_State *L, lua_Writer writer, void *data);


/*
** coroutine functions
*/
LUA_API int  (lua_yield) (lua_State *L, int nresults);
LUA_API int  (lua_resume) (lua_State *L, int narg);
LUA_API int  (lua_status) (lua_State *L);
/*
** garbage-collection function and options
*/

#define LUA_GCSTOP		0
#define LUA_GCRESTART		1
#define LUA_GCCOLLECT		2
#define LUA_GCCOUNT		3
#define LUA_GCCOUNTB		4
#define LUA_GCSTEP		5
#define LUA_GCSETPAUSE		6
#define LUA_GCSETSTEPMUL	7

LUA_API int (lua_gc) (lua_State *L, int what, int data);


/*
** miscellaneous functions
*/

LUA_API int   (lua_error) (lua_State *L);

LUA_API int   (lua_next) (lua_State *L, int idx);

LUA_API void  (lua_concat) (lua_State *L, int n);

LUA_API lua_Alloc (lua_getallocf) (lua_State *L, void **ud);
LUA_API void lua_setallocf (lua_State *L, lua_Alloc f, void *ud);


//From auxlib.h----------------------------
#if defined(LUA_COMPAT_GETN)
LUALIB_API int (luaL_getn) (lua_State *L, int t);
LUALIB_API void (luaL_setn) (lua_State *L, int t, int n);
#else
#define luaL_getn(L,i)          ((int)lua_objlen(L, i))
#define luaL_setn(L,i,j)        ((void)0)  /* no op! */
#endif

#if defined(LUA_COMPAT_OPENLIB)
#define luaI_openlib	luaL_openlib
#endif


/* extra error code for `luaL_load' */
#define LUA_ERRFILE     (LUA_ERRERR+1)


typedef struct luaL_Reg {
	const char *name;
	lua_CFunction func;
} luaL_Reg;

LUALIB_API void (luaI_openlib) (lua_State *L, const char *libname,
								const luaL_Reg *l, int nup);
LUALIB_API void (luaL_register) (lua_State *L, const char *libname,
								 const luaL_Reg *l);
LUALIB_API int (luaL_getmetafield) (lua_State *L, int obj, const char *e);
LUALIB_API int (luaL_callmeta) (lua_State *L, int obj, const char *e);
LUALIB_API int (luaL_typerror) (lua_State *L, int narg, const char *tname);
LUALIB_API int (luaL_argerror) (lua_State *L, int numarg, const char *extramsg);
LUALIB_API const char *(luaL_checklstring) (lua_State *L, int numArg,
											size_t *l);
LUALIB_API const char *(luaL_optlstring) (lua_State *L, int numArg,
										  const char *def, size_t *l);
LUALIB_API lua_Number (luaL_checknumber) (lua_State *L, int numArg);
LUALIB_API lua_Number (luaL_optnumber) (lua_State *L, int nArg, lua_Number def);

LUALIB_API lua_Integer (luaL_checkinteger) (lua_State *L, int numArg);
LUALIB_API lua_Integer (luaL_optinteger) (lua_State *L, int nArg,
										  lua_Integer def);

LUALIB_API void (luaL_checkstack) (lua_State *L, int sz, const char *msg);
LUALIB_API void (luaL_checktype) (lua_State *L, int narg, int t);
LUALIB_API void (luaL_checkany) (lua_State *L, int narg);

LUALIB_API int   (luaL_newmetatable) (lua_State *L, const char *tname);
LUALIB_API void *(luaL_checkudata) (lua_State *L, int ud, const char *tname);

LUALIB_API void (luaL_where) (lua_State *L, int lvl);
LUALIB_API int (luaL_error) (lua_State *L, const char *fmt, ...);

LUALIB_API int (luaL_checkoption) (lua_State *L, int narg, const char *def,
								   const char *const lst[]);

LUALIB_API int (luaL_ref) (lua_State *L, int t);
LUALIB_API void (luaL_unref) (lua_State *L, int t, int ref);

LUALIB_API int (luaL_loadfile) (lua_State *L, const char *filename);
LUALIB_API int (luaL_loadbuffer) (lua_State *L, const char *buff, size_t sz,
								  const char *name);
LUALIB_API int (luaL_loadstring) (lua_State *L, const char *s);

LUALIB_API lua_State *(luaL_newstate) (void);


LUALIB_API const char *(luaL_gsub) (lua_State *L, const char *s, const char *p,
									const char *r);

LUALIB_API const char *(luaL_findtable) (lua_State *L, int idx,const char *fname, int szhint);

/*
** ===============================================================
** some useful macros
** ===============================================================
*/

#define luaL_argcheck(L, cond,numarg,extramsg)	\
	((void)((cond) || luaL_argerror(L, (numarg), (extramsg))))
#define luaL_checkstring(L,n)	(luaL_checklstring(L, (n), NULL))
#define luaL_optstring(L,n,d)	(luaL_optlstring(L, (n), (d), NULL))
#define luaL_checkint(L,n)	((int)luaL_checkinteger(L, (n)))
#define luaL_optint(L,n,d)	((int)luaL_optinteger(L, (n), (d)))
#define luaL_checklong(L,n)	((long)luaL_checkinteger(L, (n)))
#define luaL_optlong(L,n,d)	((long)luaL_optinteger(L, (n), (d)))

#define luaL_typename(L,i)	lua_typename(L, lua_type(L,(i)))

#define luaL_dofile(L, fn) \
	(luaL_loadfile(L, fn) || lua_pcall(L, 0, LUA_MULTRET, 0))

#define luaL_dostring(L, s) \
	(luaL_loadstring(L, s) || lua_pcall(L, 0, LUA_MULTRET, 0))

#define luaL_getmetatable(L,n)	(lua_getfield(L, LUA_REGISTRYINDEX, (n)))

#define luaL_opt(L,f,n,d)	(lua_isnoneornil(L,(n)) ? (d) : f(L,(n)))

/*
** {======================================================
** Generic Buffer manipulation
** =======================================================
*/

typedef struct luaL_Buffer {
	char *p;			/* current position in buffer */
	int lvl;  /* number of strings in the stack (level) */
	lua_State *L;
	char buffer[LUAL_BUFFERSIZE];
} luaL_Buffer;

#define luaL_addchar(B,c) \
	((void)((B)->p < ((B)->buffer+LUAL_BUFFERSIZE) || luaL_prepbuffer(B)), \
	(*(B)->p++ = (char)(c)))

/* compatibility only */
#define luaL_putchar(B,c)	luaL_addchar(B,c)

#define luaL_addsize(B,n)	((B)->p += (n))

LUALIB_API void (luaL_buffinit) (lua_State *L, luaL_Buffer *B);
LUALIB_API char *(luaL_prepbuffer) (luaL_Buffer *B);
LUALIB_API void (luaL_addlstring) (luaL_Buffer *B, const char *s, size_t l);
LUALIB_API void (luaL_addstring) (luaL_Buffer *B, const char *s);
LUALIB_API void (luaL_addvalue) (luaL_Buffer *B);
LUALIB_API void (luaL_pushresult) (luaL_Buffer *B);
/* }====================================================== */


/* compatibility with ref system */

/* pre-defined references */
#define LUA_NOREF       (-2)
#define LUA_REFNIL      (-1)

#define lua_ref(L,lock) ((lock) ? luaL_ref(L, LUA_REGISTRYINDEX) : \
	(lua_pushstring(L, "unlocked references are obsolete"), lua_error(L), 0))

#define lua_unref(L,ref)        luaL_unref(L, LUA_REGISTRYINDEX, (ref))

#define lua_getref(L,ref)       lua_rawgeti(L, LUA_REGISTRYINDEX, (ref))


#define luaL_reg	luaL_Reg

//----end auxlib
/*
** max size of array part is 2^MAXBITS
*/
#if LUAI_BITSINT > 26
#define MAXBITS		26
#else
#define MAXBITS		(LUAI_BITSINT-2)
#endif

#define MAXASIZE	(1 << MAXBITS)


#define hashpow2(t,n)      (gnode(t, lmod((n), sizenode(t))))

#define hashstr(t,str)  hashpow2(t, (str)->tsv.hash)
#define hashboolean(t,p)        hashpow2(t, p)


/*
** for some types, it is better to avoid modulus by power of 2, as
** they tend to have many 2 factors.
*/
#define hashmod(t,n)	(gnode(t, ((n) % ((sizenode(t)-1)|1))))


#define hashpointer(t,p)	hashmod(t, IntPoint(p))


/*
** number of ints inside a lua_Number
*/
#define numints		cast_int(sizeof(lua_Number)/sizeof(int))



#define dummynode		(&dummynode_)

static const Node dummynode_ = {
	{{NULL}, LUA_TNIL},  /* value */
	{{{NULL}, LUA_TNIL, NULL}}  /* key */
};


/*
** hash for lua_Numbers
*/
static Node *hashnum (const Table *t, lua_Number n) {
	unsigned int a[numints];
	int i;
	n += 1;  /* normalize number (avoid -0) */
	lua_assert(sizeof(a) <= sizeof(n));
	memcpy(a, &n, sizeof(a));
	for (i = 1; i < numints; i++) a[0] += a[i];
	return hashmod(t, a[0]);
}



/*
** returns the `main' position of an element in a table (that is, the index
** of its hash value)
*/
static Node *mainposition (const Table *t, const TValue *key) {
	switch (ttype(key)) {
	case LUA_TNUMBER:
		return hashnum(t, nvalue(key));
	case LUA_TSTRING:
		return hashstr(t, rawtsvalue(key));
	case LUA_TBOOLEAN:
		return hashboolean(t, bvalue(key));
	case LUA_TLIGHTUSERDATA:
		return hashpointer(t, pvalue(key));
	default:
		return hashpointer(t, gcvalue(key));
	}
}


/*
** returns the index for `key' if `key' is an appropriate key to live in
** the array part of the table, -1 otherwise.
*/
static int arrayindex (const TValue *key) {
	if (ttisnumber(key)) {
		lua_Number n = nvalue(key);
		int k;
		lua_number2int(k, n);
		if (luai_numeq(cast_num(k), n))
			return k;
	}
	return -1;  /* `key' did not match some condition */
}


/*
** returns the index of a `key' for table traversals. First goes all
** elements in the array part, then elements in the hash part. The
** beginning of a traversal is signalled by -1.
*/
static int findindex (lua_State *L, Table *t, StkId key) {
	int i;
	if (ttisnil(key)) return -1;  /* first iteration */
	i = arrayindex(key);
	if (0 < i && i <= t->sizearray)  /* is `key' inside array part? */
		return i-1;  /* yes; that's the index (corrected to C) */
	else {
		Node *n = mainposition(t, key);
		do {  /* check whether `key' is somewhere in the chain */
			/* key may be dead already, but it is ok to use it in `next' */
			if (luaO_rawequalObj(key2tval(n), key) ||
				(ttype(gkey(n)) == LUA_TDEADKEY && iscollectable(key) &&
				gcvalue(gkey(n)) == gcvalue(key))) {
					i = cast_int(n - gnode(t, 0));  /* key index in hash table */
					/* hash elements are numbered after array ones */
					return i + t->sizearray;
			}
			else n = gnext(n);
		} while (n);
		luaG_runerror(L, "invalid key to " LUA_QL("next"));  /* key not found */
		return 0;  /* to avoid warnings */
	}
}


int luaH_next (lua_State *L, Table *t, StkId key) {
	int i = findindex(L, t, key);  /* find original element */
	for (i++; i < t->sizearray; i++) {  /* try first array part */
		if (!ttisnil(&t->array[i])) {  /* a non-nil value? */
			setnvalue(key, cast_num(i+1));
			setobj2s(L, key+1, &t->array[i]);
			return 1;
		}
	}
	for (i -= t->sizearray; i < sizenode(t); i++) {  /* then hash part */
		if (!ttisnil(gval(gnode(t, i)))) {  /* a non-nil value? */
			setobj2s(L, key, key2tval(gnode(t, i)));
			setobj2s(L, key+1, gval(gnode(t, i)));
			return 1;
		}
	}
	return 0;  /* no more elements */
}


/*
** {=============================================================
** Rehash
** ==============================================================
*/


static int computesizes (int nums[], int *narray) {
	int i;
	int twotoi;  /* 2^i */
	int a = 0;  /* number of elements smaller than 2^i */
	int na = 0;  /* number of elements to go to array part */
	int n = 0;  /* optimal size for array part */
	for (i = 0, twotoi = 1; twotoi/2 < *narray; i++, twotoi *= 2) {
		if (nums[i] > 0) {
			a += nums[i];
			if (a > twotoi/2) {  /* more than half elements present? */
				n = twotoi;  /* optimal size (till now) */
				na = a;  /* all elements smaller than n will go to array part */
			}
		}
		if (a == *narray) break;  /* all elements already counted */
	}
	*narray = n;
	lua_assert(*narray/2 <= na && na <= *narray);
	return na;
}


static int countint (const TValue *key, int *nums) {
	int k = arrayindex(key);
	if (0 < k && k <= MAXASIZE) {  /* is `key' an appropriate array index? */
		nums[ceillog2(k)]++;  /* count as such */
		return 1;
	}
	else
		return 0;
}


static int numusearray (const Table *t, int *nums) {
	int lg;
	int ttlg;  /* 2^lg */
	int ause = 0;  /* summation of `nums' */
	int i = 1;  /* count to traverse all array keys */
	for (lg=0, ttlg=1; lg<=MAXBITS; lg++, ttlg*=2) {  /* for each slice */
		int lc = 0;  /* counter */
		int lim = ttlg;
		if (lim > t->sizearray) {
			lim = t->sizearray;  /* adjust upper limit */
			if (i > lim)
				break;  /* no more elements to count */
		}
		/* count elements in range (2^(lg-1), 2^lg] */
		for (; i <= lim; i++) {
			if (!ttisnil(&t->array[i-1]))
				lc++;
		}
		nums[lg] += lc;
		ause += lc;
	}
	return ause;
}


static int numusehash (const Table *t, int *nums, int *pnasize) {
	int totaluse = 0;  /* total number of elements */
	int ause = 0;  /* summation of `nums' */
	int i = sizenode(t);
	while (i--) {
		Node *n = &t->node[i];
		if (!ttisnil(gval(n))) {
			ause += countint(key2tval(n), nums);
			totaluse++;
		}
	}
	*pnasize += ause;
	return totaluse;
}


static void setarrayvector (lua_State *L, Table *t, int size) {
	int i;
	luaM_reallocvector(L, t->array, t->sizearray, size, TValue);
	for (i=t->sizearray; i<size; i++)
		setnilvalue(&t->array[i]);
	t->sizearray = size;
}


static void setnodevector (lua_State *L, Table *t, int size) {
	int lsize;
	if (size == 0) {  /* no elements to hash part? */
		t->node = cast(Node *, dummynode);  /* use common `dummynode' */
		lsize = 0;
	}
	else {
		int i;
		lsize = ceillog2(size);
		if (lsize > MAXBITS)
			luaG_runerror(L, "table overflow");
		size = twoto(lsize);
		t->node = luaM_newvector(L, size, Node);
		for (i=0; i<size; i++) {
			Node *n = gnode(t, i);
			gnext(n) = NULL;
			setnilvalue(gkey(n));
			setnilvalue(gval(n));
		}
	}
	t->lsizenode = cast_byte(lsize);
	t->lastfree = gnode(t, size);  /* all positions are free */
}


static void resize (lua_State *L, Table *t, int nasize, int nhsize) {
	int i;
	int oldasize = t->sizearray;
	int oldhsize = t->lsizenode;
	Node *nold = t->node;  /* save old hash ... */
	if (nasize > oldasize)  /* array part must grow? */
		setarrayvector(L, t, nasize);
	/* create new hash part with appropriate size */
	setnodevector(L, t, nhsize);  
	if (nasize < oldasize) {  /* array part must shrink? */
		t->sizearray = nasize;
		/* re-insert elements from vanishing slice */
		for (i=nasize; i<oldasize; i++) {
			if (!ttisnil(&t->array[i]))
				setobjt2t(L, luaH_setnum(L, t, i+1), &t->array[i]);
		}
		/* shrink array */
		luaM_reallocvector(L, t->array, oldasize, nasize, TValue);
	}
	/* re-insert elements from hash part */
	for (i = twoto(oldhsize) - 1; i >= 0; i--) {
		Node *old = nold+i;
		if (!ttisnil(gval(old)))
			setobjt2t(L, luaH_set(L, t, key2tval(old)), gval(old));
	}
	if (nold != dummynode)
		luaM_freearray(L, nold, twoto(oldhsize), Node);  /* free old array */
}


void luaH_resizearray (lua_State *L, Table *t, int nasize) {
	int nsize = (t->node == dummynode) ? 0 : sizenode(t);
	resize(L, t, nasize, nsize);
}


static void rehash (lua_State *L, Table *t, const TValue *ek) {
	int nasize, na;
	int nums[MAXBITS+1];  /* nums[i] = number of keys between 2^(i-1) and 2^i */
	int i;
	int totaluse;
	for (i=0; i<=MAXBITS; i++) nums[i] = 0;  /* reset counts */
	nasize = numusearray(t, nums);  /* count keys in array part */
	totaluse = nasize;  /* all those keys are integer keys */
	totaluse += numusehash(t, nums, &nasize);  /* count keys in hash part */
	/* count extra key */
	nasize += countint(ek, nums);
	totaluse++;
	/* compute new size for array part */
	na = computesizes(nums, &nasize);
	/* resize the table to new computed sizes */
	resize(L, t, nasize, totaluse - na);
}



/*
** }=============================================================
*/


Table *luaH_new (lua_State *L, int narray, int nhash) {
	Table *t = luaM_new(L, Table);
	luaC_link(L, obj2gco(t), LUA_TTABLE);
	t->metatable = NULL;
	t->flags = cast_byte(~0);
	/* temporary values (kept only if some malloc fails) */
	t->array = NULL;
	t->sizearray = 0;
	t->lsizenode = 0;
	t->node = cast(Node *, dummynode);
	setarrayvector(L, t, narray);
	setnodevector(L, t, nhash);
	return t;
}


void luaH_free (lua_State *L, Table *t) {
	if (t->node != dummynode)
		luaM_freearray(L, t->node, sizenode(t), Node);
	luaM_freearray(L, t->array, t->sizearray, TValue);
	luaM_free(L, t);
}


static Node *getfreepos (Table *t) {
	while (t->lastfree-- > t->node) {
		if (ttisnil(gkey(t->lastfree)))
			return t->lastfree;
	}
	return NULL;  /* could not find a free place */
}



/*
** inserts a new key into a hash table; first, check whether key's main 
** position is free. If not, check whether colliding node is in its main 
** position or not: if it is not, move colliding node to an empty place and 
** put new key in its main position; otherwise (colliding node is in its main 
** position), new key goes to an empty position. 
*/
static TValue *newkey (lua_State *L, Table *t, const TValue *key) {
	Node *mp = mainposition(t, key);
	if (!ttisnil(gval(mp)) || mp == dummynode) {
		Node *othern;
		Node *n = getfreepos(t);  /* get a free place */
		if (n == NULL) {  /* cannot find a free place? */
			rehash(L, t, key);  /* grow table */
			return luaH_set(L, t, key);  /* re-insert key into grown table */
		}
		lua_assert(n != dummynode);
		othern = mainposition(t, key2tval(mp));
		if (othern != mp) {  /* is colliding node out of its main position? */
			/* yes; move colliding node into free position */
			while (gnext(othern) != mp) othern = gnext(othern);  /* find previous */
			gnext(othern) = n;  /* redo the chain with `n' in place of `mp' */
			*n = *mp;  /* copy colliding node into free pos. (mp->next also goes) */
			gnext(mp) = NULL;  /* now `mp' is free */
			setnilvalue(gval(mp));
		}
		else {  /* colliding node is in its own main position */
			/* new node will go into free position */
			gnext(n) = gnext(mp);  /* chain new position */
			gnext(mp) = n;
			mp = n;
		}
	}
	gkey(mp)->value = key->value; gkey(mp)->tt = key->tt;
	luaC_barriert(L, t, key);
	lua_assert(ttisnil(gval(mp)));
	return gval(mp);
}


/*
** search function for integers
*/
const TValue *luaH_getnum (Table *t, int key) {
	/* (1 <= key && key <= t->sizearray) */
	if (cast(unsigned int, key-1) < cast(unsigned int, t->sizearray))
		return &t->array[key-1];
	else {
		lua_Number nk = cast_num(key);
		Node *n = hashnum(t, nk);
		do {  /* check whether `key' is somewhere in the chain */
			if (ttisnumber(gkey(n)) && luai_numeq(nvalue(gkey(n)), nk))
				return gval(n);  /* that's it */
			else n = gnext(n);
		} while (n);
		return luaO_nilobject;
	}
}


/*
** search function for strings
*/
const TValue *luaH_getstr (Table *t, TString *key) {
	Node *n = hashstr(t, key);
	do {  /* check whether `key' is somewhere in the chain */
		if (ttisstring(gkey(n)) && rawtsvalue(gkey(n)) == key)
			return gval(n);  /* that's it */
		else n = gnext(n);
	} while (n);
	return luaO_nilobject;
}


/*
** main search function
*/
const TValue *luaH_get (Table *t, const TValue *key) {
	switch (ttype(key)) {
	case LUA_TNIL: return luaO_nilobject;
	case LUA_TSTRING: return luaH_getstr(t, rawtsvalue(key));
	case LUA_TNUMBER: {
		int k;
		lua_Number n = nvalue(key);
		lua_number2int(k, n);
		if (luai_numeq(cast_num(k), nvalue(key))) /* index is int? */
			return luaH_getnum(t, k);  /* use specialized version */
		/* else go through */
					  }
	default: {
		Node *n = mainposition(t, key);
		do {  /* check whether `key' is somewhere in the chain */
			if (luaO_rawequalObj(key2tval(n), key))
				return gval(n);  /* that's it */
			else n = gnext(n);
		} while (n);
		return luaO_nilobject;
			 }
	}
}


TValue *luaH_set (lua_State *L, Table *t, const TValue *key) {
	const TValue *p = luaH_get(t, key);
	t->flags = 0;
	if (p != luaO_nilobject)
		return cast(TValue *, p);
	else {
		if (ttisnil(key)) luaG_runerror(L, "table index is nil");
		else if (ttisnumber(key) && luai_numisnan(nvalue(key)))
			luaG_runerror(L, "table index is NaN");
		return newkey(L, t, key);
	}
}


TValue *luaH_setnum (lua_State *L, Table *t, int key) {
	const TValue *p = luaH_getnum(t, key);
	if (p != luaO_nilobject)
		return cast(TValue *, p);
	else {
		TValue k;
		setnvalue(&k, cast_num(key));
		return newkey(L, t, &k);
	}
}


TValue *luaH_setstr (lua_State *L, Table *t, TString *key) {
	const TValue *p = luaH_getstr(t, key);
	if (p != luaO_nilobject)
		return cast(TValue *, p);
	else {
		TValue k;
		setsvalue(L, &k, key);
		return newkey(L, t, &k);
	}
}


static int unbound_search (Table *t, unsigned int j) {
	unsigned int i = j;  /* i is zero or a present index */
	j++;
	/* find `i' and `j' such that i is present and j is not */
	while (!ttisnil(luaH_getnum(t, j))) {
		i = j;
		j *= 2;
		if (j > cast(unsigned int, MAX_INT)) {  /* overflow? */
			/* table was built with bad purposes: resort to linear search */
			i = 1;
			while (!ttisnil(luaH_getnum(t, i))) i++;
			return i - 1;
		}
	}
	/* now do a binary search between them */
	while (j - i > 1) {
		unsigned int m = (i+j)/2;
		if (ttisnil(luaH_getnum(t, m))) j = m;
		else i = m;
	}
	return i;
}


/*
** Try to find a boundary in table `t'. A `boundary' is an integer index
** such that t[i] is non-nil and t[i+1] is nil (and 0 if t[1] is nil).
*/
int luaH_getn (Table *t) {
	unsigned int j = t->sizearray;
	if (j > 0 && ttisnil(&t->array[j - 1])) {
		/* there is a boundary in the array part: (binary) search for it */
		unsigned int i = 0;
		while (j - i > 1) {
			unsigned int m = (i+j)/2;
			if (ttisnil(&t->array[m - 1])) j = m;
			else i = m;
		}
		return i;
	}
	/* else must find a boundary in hash part */
	else if (t->node == dummynode)  /* hash part is empty? */
		return j;  /* that is easy... */
	else return unbound_search(t, j);
}



#if defined(LUA_DEBUG)

Node *luaH_mainposition (const Table *t, const TValue *key) {
	return mainposition(t, key);
}

int luaH_isdummy (Node *n) { return n == dummynode; }

#endif

//-------------------------------------------------------------ltablib.c----------------------------------------------------

#if 0
#define aux_getn(L,n)	(luaL_checktype(L, n, LUA_TTABLE), luaL_getn(L, n))


static int foreachi (lua_State *L) {
	int i;
	int n = aux_getn(L, 1);
	luaL_checktype(L, 2, LUA_TFUNCTION);
	for (i=1; i <= n; i++) {
		lua_pushvalue(L, 2);  /* function */
		lua_pushinteger(L, i);  /* 1st argument */
		lua_rawgeti(L, 1, i);  /* 2nd argument */
		lua_call(L, 2, 1);
		if (!lua_isnil(L, -1))
			return 1;
		lua_pop(L, 1);  /* remove nil result */
	}
	return 0;
}


static int foreach (lua_State *L) {
	luaL_checktype(L, 1, LUA_TTABLE);
	luaL_checktype(L, 2, LUA_TFUNCTION);
	lua_pushnil(L);  /* first key */
	while (lua_next(L, 1)) {
		lua_pushvalue(L, 2);  /* function */
		lua_pushvalue(L, -3);  /* key */
		lua_pushvalue(L, -3);  /* value */
		lua_call(L, 2, 1);
		if (!lua_isnil(L, -1))
			return 1;
		lua_pop(L, 2);  /* remove value and result */
	}
	return 0;
}


static int maxn (lua_State *L) {
	lua_Number max = 0;
	luaL_checktype(L, 1, LUA_TTABLE);
	lua_pushnil(L);  /* first key */
	while (lua_next(L, 1)) {
		lua_pop(L, 1);  /* remove value */
		if (lua_type(L, -1) == LUA_TNUMBER) {
			lua_Number v = lua_tonumber(L, -1);
			if (v > max) max = v;
		}
	}
	lua_pushnumber(L, max);
	return 1;
}


static int getn (lua_State *L) {
	lua_pushinteger(L, aux_getn(L, 1));
	return 1;
}


static int setn (lua_State *L) {
	luaL_checktype(L, 1, LUA_TTABLE);
#ifndef luaL_setn
	luaL_setn(L, 1, luaL_checkint(L, 2));
#else
	luaL_error(L, LUA_QL("setn") " is obsolete");
#endif
	lua_pushvalue(L, 1);
	return 1;
}


static int tinsert (lua_State *L) {
	int e = aux_getn(L, 1) + 1;  /* first empty element */
	int pos;  /* where to insert new element */
	switch (lua_gettop(L)) {
	case 2: {  /* called with only 2 arguments */
		pos = e;  /* insert new element at the end */
		break;
			}
	case 3: {
		int i;
		pos = luaL_checkint(L, 2);  /* 2nd argument is the position */
		if (pos > e) e = pos;  /* `grow' array if necessary */
		for (i = e; i > pos; i--) {  /* move up elements */
			lua_rawgeti(L, 1, i-1);
			lua_rawseti(L, 1, i);  /* t[i] = t[i-1] */
		}
		break;
			}
	default: {
		return luaL_error(L, "wrong number of arguments to " LUA_QL("insert"));
			 }
	}
	luaL_setn(L, 1, e);  /* new size */
	lua_rawseti(L, 1, pos);  /* t[pos] = v */
	return 0;
}


static int tremove (lua_State *L) {
	int e = aux_getn(L, 1);
	int pos = luaL_optint(L, 2, e);
	if (e == 0) return 0;  /* table is `empty' */
	luaL_setn(L, 1, e - 1);  /* t.n = n-1 */
	lua_rawgeti(L, 1, pos);  /* result = t[pos] */
	for ( ;pos<e; pos++) {
		lua_rawgeti(L, 1, pos+1);
		lua_rawseti(L, 1, pos);  /* t[pos] = t[pos+1] */
	}
	lua_pushnil(L);
	lua_rawseti(L, 1, e);  /* t[e] = nil */
	return 1;
}


static int tconcat (lua_State *L) {
	luaL_Buffer b;
	size_t lsep;
	int i, last;
	const char *sep = luaL_optlstring(L, 2, "", &lsep);
	luaL_checktype(L, 1, LUA_TTABLE);
	i = luaL_optint(L, 3, 1);
	last = luaL_opt(L, luaL_checkint, 4, luaL_getn(L, 1));
	luaL_buffinit(L, &b);
	for (; i <= last; i++) {
		lua_rawgeti(L, 1, i);
		luaL_argcheck(L, lua_isstring(L, -1), 1, "table contains non-strings");
		luaL_addvalue(&b);
		if (i != last)
			luaL_addlstring(&b, sep, lsep);
	}
	luaL_pushresult(&b);
	return 1;
}



/*
** {======================================================
** Quicksort
** (based on `Algorithms in MODULA-3', Robert Sedgewick;
**  Addison-Wesley, 1993.)
*/


static void set2 (lua_State *L, int i, int j) {
	lua_rawseti(L, 1, i);
	lua_rawseti(L, 1, j);
}

static int sort_comp (lua_State *L, int a, int b) {
	if (!lua_isnil(L, 2)) {  /* function? */
		int res;
		lua_pushvalue(L, 2);
		lua_pushvalue(L, a-1);  /* -1 to compensate function */
		lua_pushvalue(L, b-2);  /* -2 to compensate function and `a' */
		lua_call(L, 2, 1);
		res = lua_toboolean(L, -1);
		lua_pop(L, 1);
		return res;
	}
	else  /* a < b? */
		return lua_lessthan(L, a, b);
}

static void auxsort (lua_State *L, int l, int u) {
	while (l < u) {  /* for tail recursion */
		int i, j;
		/* sort elements a[l], a[(l+u)/2] and a[u] */
		lua_rawgeti(L, 1, l);
		lua_rawgeti(L, 1, u);
		if (sort_comp(L, -1, -2))  /* a[u] < a[l]? */
			set2(L, l, u);  /* swap a[l] - a[u] */
		else
			lua_pop(L, 2);
		if (u-l == 1) break;  /* only 2 elements */
		i = (l+u)/2;
		lua_rawgeti(L, 1, i);
		lua_rawgeti(L, 1, l);
		if (sort_comp(L, -2, -1))  /* a[i]<a[l]? */
			set2(L, i, l);
		else {
			lua_pop(L, 1);  /* remove a[l] */
			lua_rawgeti(L, 1, u);
			if (sort_comp(L, -1, -2))  /* a[u]<a[i]? */
				set2(L, i, u);
			else
				lua_pop(L, 2);
		}
		if (u-l == 2) break;  /* only 3 elements */
		lua_rawgeti(L, 1, i);  /* Pivot */
		lua_pushvalue(L, -1);
		lua_rawgeti(L, 1, u-1);
		set2(L, i, u-1);
		/* a[l] <= P == a[u-1] <= a[u], only need to sort from l+1 to u-2 */
		i = l; j = u-1;
		for (;;) {  /* invariant: a[l..i] <= P <= a[j..u] */
			/* repeat ++i until a[i] >= P */
			while (lua_rawgeti(L, 1, ++i), sort_comp(L, -1, -2)) {
				if (i>u) luaL_error(L, "invalid order function for sorting");
				lua_pop(L, 1);  /* remove a[i] */
			}
			/* repeat --j until a[j] <= P */
			while (lua_rawgeti(L, 1, --j), sort_comp(L, -3, -1)) {
				if (j<l) luaL_error(L, "invalid order function for sorting");
				lua_pop(L, 1);  /* remove a[j] */
			}
			if (j<i) {
				lua_pop(L, 3);  /* pop pivot, a[i], a[j] */
				break;
			}
			set2(L, i, j);
		}
		lua_rawgeti(L, 1, u-1);
		lua_rawgeti(L, 1, i);
		set2(L, u-1, i);  /* swap pivot (a[u-1]) with a[i] */
		/* a[l..i-1] <= a[i] == P <= a[i+1..u] */
		/* adjust so that smaller half is in [j..i] and larger one in [l..u] */
		if (i-l < u-i) {
			j=l; i=i-1; l=i+2;
		}
		else {
			j=i+1; i=u; u=j-2;
		}
		auxsort(L, j, i);  /* call recursively the smaller one */
	}  /* repeat the routine for the larger one */
}

static int sort (lua_State *L) {
	int n = aux_getn(L, 1);
	luaL_checkstack(L, 40, "");  /* assume array is smaller than 2^40 */
	if (!lua_isnoneornil(L, 2))  /* is there a 2nd argument? */
		luaL_checktype(L, 2, LUA_TFUNCTION);
	lua_settop(L, 2);  /* make sure there is two arguments */
	auxsort(L, 1, n);
	return 0;
}

/* }====================================================== */

static const luaL_Reg tab_funcs[] = {
	{"concat", tconcat},
	{"foreach", foreach},
	{"foreachi", foreachi},
	{"getn", getn},
	{"maxn", maxn},
	{"insert", tinsert},
	{"remove", tremove},
	{"setn", setn},
	{"sort", sort},
	{NULL, NULL}
};

LUALIB_API int luaopen_table (lua_State *L) {
	luaL_register(L, LUA_TABLIBNAME, tab_funcs);
	return 1;
}
#endif

//-------------------------------------------------------------lgc.c--------------------------------------------------------
//from lstate.h
LUAI_FUNC lua_State *luaE_newthread (lua_State *L);
LUAI_FUNC void luaE_freethread (lua_State *L, lua_State *L1);

#define GCSTEPSIZE	1024u
#define GCSWEEPMAX	40
#define GCSWEEPCOST	10
#define GCFINALIZECOST	100


#define maskmarks	cast_byte(~(bitmask(BLACKBIT)|WHITEBITS))

#define makewhite(g,x)	\
	((x)->gch.marked = cast_byte(((x)->gch.marked & maskmarks) | luaC_white(g)))

#define white2gray(x)	reset2bits((x)->gch.marked, WHITE0BIT, WHITE1BIT)
#define black2gray(x)	resetbit((x)->gch.marked, BLACKBIT)

#define stringmark(s)	reset2bits((s)->tsv.marked, WHITE0BIT, WHITE1BIT)


#define isfinalized(u)		testbit((u)->marked, FINALIZEDBIT)
#define markfinalized(u)	l_setbit((u)->marked, FINALIZEDBIT)


#define KEYWEAK         bitmask(KEYWEAKBIT)
#define VALUEWEAK       bitmask(VALUEWEAKBIT)



#define markvalue(g,o) { checkconsistency(o); \
	if (iscollectable(o) && iswhite(gcvalue(o))) reallymarkobject(g,gcvalue(o)); }

#define markobject(g,t) { if (iswhite(obj2gco(t))) \
	reallymarkobject(g, obj2gco(t)); }


#define setthreshold(g)  (g->GCthreshold = (g->estimate/100) * g->gcpause)

static void removeentry (Node *n) {
	lua_assert(ttisnil(gval(n)));
	if (iscollectable(gkey(n)))
		setttype(gkey(n), LUA_TDEADKEY);  /* dead key; remove it */
}


static void reallymarkobject (global_State *g, GCObject *o) {
	lua_assert(iswhite(o) && !isdead(g, o));
	white2gray(o);
	switch (o->gch.tt) {
	case LUA_TSTRING: {
		return;
					  }
	case LUA_TUSERDATA: {
		Table *mt = gco2u(o)->metatable;
		gray2black(o);  /* udata are never gray */
		if (mt) markobject(g, mt);
		markobject(g, gco2u(o)->env);
		return;
						}
	case LUA_TUPVAL: {
		UpVal *uv = gco2uv(o);
		markvalue(g, uv->v);
		if (uv->v == &uv->u.value)  /* closed? */
			gray2black(o);  /* open upvalues are never black */
		return;
					 }
	case LUA_TFUNCTION: {
		gco2cl(o)->c.gclist = g->gray;
		g->gray = o;
		break;
						}
	case LUA_TTABLE: {
		gco2h(o)->gclist = g->gray;
		g->gray = o;
		break;
					 }
	case LUA_TTHREAD: {
		gco2th(o)->gclist = g->gray;
		g->gray = o;
		break;
					  }
	case LUA_TPROTO: {
		gco2p(o)->gclist = g->gray;
		g->gray = o;
		break;
					 }
	default: lua_assert(0);
	}
}


static void marktmu (global_State *g) {
	GCObject *u = g->tmudata;
	if (u) {
		do {
			u = u->gch.next;
			makewhite(g, u);  /* may be marked, if left from previous GC */
			reallymarkobject(g, u);
		} while (u != g->tmudata);
	}
}


/* move `dead' udata that need finalization to list `tmudata' */
size_t luaC_separateudata (lua_State *L, int all) {
	global_State *g = G(L);
	size_t deadmem = 0;
	GCObject **p = &g->mainthread->next;
	GCObject *curr;
	while ((curr = *p) != NULL) {
		if (!(iswhite(curr) || all) || isfinalized(gco2u(curr)))
			p = &curr->gch.next;  /* don't bother with them */
		else if (fasttm(L, gco2u(curr)->metatable, TM_GC) == NULL) {
			markfinalized(gco2u(curr));  /* don't need finalization */
			p = &curr->gch.next;
		}
		else {  /* must call its gc method */
			deadmem += sizeudata(gco2u(curr));
			markfinalized(gco2u(curr));
			*p = curr->gch.next;
			/* link `curr' at the end of `tmudata' list */
			if (g->tmudata == NULL)  /* list is empty? */
				g->tmudata = curr->gch.next = curr;  /* creates a circular list */
			else {
				curr->gch.next = g->tmudata->gch.next;
				g->tmudata->gch.next = curr;
				g->tmudata = curr;
			}
		}
	}
	return deadmem;
}


static int traversetable (global_State *g, Table *h) {
	int i;
	int weakkey = 0;
	int weakvalue = 0;
	const TValue *mode;
	if (h->metatable)
		markobject(g, h->metatable);
	mode = gfasttm(g, h->metatable, TM_MODE);
	if (mode && ttisstring(mode)) {  /* is there a weak mode? */
		weakkey = (strchr(svalue(mode), 'k') != NULL);
		weakvalue = (strchr(svalue(mode), 'v') != NULL);
		if (weakkey || weakvalue) {  /* is really weak? */
			h->marked &= ~(KEYWEAK | VALUEWEAK);  /* clear bits */
			h->marked |= cast_byte((weakkey << KEYWEAKBIT) |
				(weakvalue << VALUEWEAKBIT));
			h->gclist = g->weak;  /* must be cleared after GC, ... */
			g->weak = obj2gco(h);  /* ... so put in the appropriate list */
		}
	}
	if (weakkey && weakvalue) return 1;
	if (!weakvalue) {
		i = h->sizearray;
		while (i--)
			markvalue(g, &h->array[i]);
	}
	i = sizenode(h);
	while (i--) {
		Node *n = gnode(h, i);
		lua_assert(ttype(gkey(n)) != LUA_TDEADKEY || ttisnil(gval(n)));
		if (ttisnil(gval(n)))
			removeentry(n);  /* remove empty entries */
		else {
			lua_assert(!ttisnil(gkey(n)));
			if (!weakkey) markvalue(g, gkey(n));
			if (!weakvalue) markvalue(g, gval(n));
		}
	}
	return weakkey || weakvalue;
}


/*
** All marks are conditional because a GC may happen while the
** prototype is still being created
*/
static void traverseproto (global_State *g, Proto *f) {
	int i;
	if (f->source) stringmark(f->source);
	for (i=0; i<f->sizek; i++)  /* mark literals */
		markvalue(g, &f->k[i]);
	for (i=0; i<f->sizeupvalues; i++) {  /* mark upvalue names */
		if (f->upvalues[i])
			stringmark(f->upvalues[i]);
	}
	for (i=0; i<f->sizep; i++) {  /* mark nested protos */
		if (f->p[i])
			markobject(g, f->p[i]);
	}
	for (i=0; i<f->sizelocvars; i++) {  /* mark local-variable names */
		if (f->locvars[i].varname)
			stringmark(f->locvars[i].varname);
	}
}



static void traverseclosure (global_State *g, Closure *cl) {
	markobject(g, cl->c.env);
	if (cl->c.isC) {
		int i;
		for (i=0; i<cl->c.nupvalues; i++)  /* mark its upvalues */
			markvalue(g, &cl->c.upvalue[i]);
	}
	else {
		int i;
		lua_assert(cl->l.nupvalues == cl->l.p->nups);
		markobject(g, cl->l.p);
		for (i=0; i<cl->l.nupvalues; i++)  /* mark its upvalues */
			markobject(g, cl->l.upvals[i]);
	}
}


static void checkstacksizes (lua_State *L, StkId max) {
	int ci_used = cast_int(L->ci - L->base_ci);  /* number of `ci' in use */
	int s_used = cast_int(max - L->stack);  /* part of stack in use */
	if (L->size_ci > LUAI_MAXCALLS)  /* handling overflow? */
		return;  /* do not touch the stacks */
	if (4*ci_used < L->size_ci && 2*BASIC_CI_SIZE < L->size_ci)
		luaD_reallocCI(L, L->size_ci/2);  /* still big enough... */
	condhardstacktests(luaD_reallocCI(L, ci_used + 1));
	if (4*s_used < L->stacksize &&
		2*(BASIC_STACK_SIZE+EXTRA_STACK) < L->stacksize)
		luaD_reallocstack(L, L->stacksize/2);  /* still big enough... */
	condhardstacktests(luaD_reallocstack(L, s_used));
}


static void traversestack (global_State *g, lua_State *l) {
	StkId o, lim;
	CallInfo *ci;
	markvalue(g, gt(l));
	lim = l->top;
	for (ci = l->base_ci; ci <= l->ci; ci++) {
		lua_assert(ci->top <= l->stack_last);
		if (lim < ci->top) lim = ci->top;
	}
	for (o = l->stack; o < l->top; o++)
		markvalue(g, o);
	for (; o <= lim; o++)
		setnilvalue(o);
	checkstacksizes(l, lim);
}


/*
** traverse one gray object, turning it to black.
** Returns `quantity' traversed.
*/
static l_mem propagatemark (global_State *g) {
	GCObject *o = g->gray;
	lua_assert(isgray(o));
	gray2black(o);
	switch (o->gch.tt) {
	case LUA_TTABLE: {
		Table *h = gco2h(o);
		g->gray = h->gclist;
		if (traversetable(g, h))  /* table is weak? */
			black2gray(o);  /* keep it gray */
		return sizeof(Table) + sizeof(TValue) * h->sizearray +
			sizeof(Node) * sizenode(h);
					 }
	case LUA_TFUNCTION: {
		Closure *cl = gco2cl(o);
		g->gray = cl->c.gclist;
		traverseclosure(g, cl);
		return (cl->c.isC) ? sizeCclosure(cl->c.nupvalues) :
			sizeLclosure(cl->l.nupvalues);
						}
	case LUA_TTHREAD: {
		lua_State *th = gco2th(o);
		g->gray = th->gclist;
		th->gclist = g->grayagain;
		g->grayagain = o;
		black2gray(o);
		traversestack(g, th);
		return sizeof(lua_State) + sizeof(TValue) * th->stacksize +
			sizeof(CallInfo) * th->size_ci;
					  }
	case LUA_TPROTO: {
		Proto *p = gco2p(o);
		g->gray = p->gclist;
		traverseproto(g, p);
		return sizeof(Proto) + sizeof(Instruction) * p->sizecode +
			sizeof(Proto *) * p->sizep +
			sizeof(TValue) * p->sizek + 
			sizeof(int) * p->sizelineinfo +
			sizeof(LocVar) * p->sizelocvars +
			sizeof(TString *) * p->sizeupvalues;
					 }
	default: lua_assert(0); return 0;
	}
}


static size_t propagateall (global_State *g) {
	size_t m = 0;
	while (g->gray) m += propagatemark(g);
	return m;
}


/*
** The next function tells whether a key or value can be cleared from
** a weak table. Non-collectable objects are never removed from weak
** tables. Strings behave as `values', so are never removed too. for
** other objects: if really collected, cannot keep them; for userdata
** being finalized, keep them in keys, but not in values
*/
static int iscleared (const TValue *o, int iskey) {
	if (!iscollectable(o)) return 0;
	if (ttisstring(o)) {
		stringmark(rawtsvalue(o));  /* strings are `values', so are never weak */
		return 0;
	}
	return iswhite(gcvalue(o)) ||
		(ttisuserdata(o) && (!iskey && isfinalized(uvalue(o))));
}


/*
** clear collected entries from weaktables
*/
static void cleartable (GCObject *l) {
	while (l) {
		Table *h = gco2h(l);
		int i = h->sizearray;
		lua_assert(testbit(h->marked, VALUEWEAKBIT) ||
			testbit(h->marked, KEYWEAKBIT));
		if (testbit(h->marked, VALUEWEAKBIT)) {
			while (i--) {
				TValue *o = &h->array[i];
				if (iscleared(o, 0))  /* value was collected? */
					setnilvalue(o);  /* remove value */
			}
		}
		i = sizenode(h);
		while (i--) {
			Node *n = gnode(h, i);
			if (!ttisnil(gval(n)) &&  /* non-empty entry? */
				(iscleared(key2tval(n), 1) || iscleared(gval(n), 0))) {
					setnilvalue(gval(n));  /* remove value ... */
					removeentry(n);  /* remove entry from table */
			}
		}
		l = h->gclist;
	}
}


static void freeobj (lua_State *L, GCObject *o) {
	switch (o->gch.tt) {
	case LUA_TPROTO: luaF_freeproto(L, gco2p(o)); break;
	case LUA_TFUNCTION: luaF_freeclosure(L, gco2cl(o)); break;
	case LUA_TUPVAL: luaF_freeupval(L, gco2uv(o)); break;
	case LUA_TTABLE: luaH_free(L, gco2h(o)); break;
	case LUA_TTHREAD: {
		lua_assert(gco2th(o) != L && gco2th(o) != G(L)->mainthread);
		luaE_freethread(L, gco2th(o));
		break;
					  }
	case LUA_TSTRING: {
		G(L)->strt.nuse--;
		luaM_freemem(L, o, sizestring(gco2ts(o)));
		break;
					  }
	case LUA_TUSERDATA: {
		luaM_freemem(L, o, sizeudata(gco2u(o)));
		break;
						}
	default: lua_assert(0);
	}
}



#define sweepwholelist(L,p)	sweeplist(L,p,MAX_LUMEM)


static GCObject **sweeplist (lua_State *L, GCObject **p, lu_mem count) {
	GCObject *curr;
	global_State *g = G(L);
	int deadmask = otherwhite(g);
	while ((curr = *p) != NULL && count-- > 0) {
		if (curr->gch.tt == LUA_TTHREAD)  /* sweep open upvalues of each thread */
			sweepwholelist(L, &gco2th(curr)->openupval);
		if ((curr->gch.marked ^ WHITEBITS) & deadmask) {  /* not dead? */
			lua_assert(!isdead(g, curr) || testbit(curr->gch.marked, FIXEDBIT));
			makewhite(g, curr);  /* make it white (for next cycle) */
			p = &curr->gch.next;
		}
		else {  /* must erase `curr' */
			lua_assert(isdead(g, curr) || deadmask == bitmask(SFIXEDBIT));
			*p = curr->gch.next;
			if (curr == g->rootgc)  /* is the first element of the list? */
				g->rootgc = curr->gch.next;  /* adjust first */
			freeobj(L, curr);
		}
	}
	return p;
}


static void checkSizes (lua_State *L) {
	global_State *g = G(L);
	/* check size of string hash */
	if (g->strt.nuse < cast(lu_int32, g->strt.size/4) &&
		g->strt.size > MINSTRTABSIZE*2)
		luaS_resize(L, g->strt.size/2);  /* table is too big */
	/* check size of buffer */
	if (luaZ_sizebuffer(&g->buff) > LUA_MINBUFFER*2) {  /* buffer too big? */
		size_t newsize = luaZ_sizebuffer(&g->buff) / 2;
		luaZ_resizebuffer(L, &g->buff, newsize);
	}
}


static void GCTM (lua_State *L) {
	global_State *g = G(L);
	GCObject *o = g->tmudata->gch.next;  /* get first element */
	Udata *udata = rawgco2u(o);
	const TValue *tm;
	/* remove udata from `tmudata' */
	if (o == g->tmudata)  /* last element? */
		g->tmudata = NULL;
	else
		g->tmudata->gch.next = udata->uv.next;
	udata->uv.next = g->mainthread->next;  /* return it to `root' list */
	g->mainthread->next = o;
	makewhite(g, o);
	tm = fasttm(L, udata->uv.metatable, TM_GC);
	if (tm != NULL) {
		lu_byte oldah = L->allowhook;
		lu_mem oldt = g->GCthreshold;
		L->allowhook = 0;  /* stop debug hooks during GC tag method */
		g->GCthreshold = 2*g->totalbytes;  /* avoid GC steps */
		setobj2s(L, L->top, tm);
		setuvalue(L, L->top+1, udata);
		L->top += 2;
		luaD_call(L, L->top - 2, 0);
		L->allowhook = oldah;  /* restore hooks */
		g->GCthreshold = oldt;  /* restore threshold */
	}
}


/*
** Call all GC tag methods
*/
void luaC_callGCTM (lua_State *L) {
	while (G(L)->tmudata)
		GCTM(L);
}


void luaC_freeall (lua_State *L) {
	global_State *g = G(L);
	int i;
	g->currentwhite = WHITEBITS | bitmask(SFIXEDBIT);  /* mask to collect all elements */
	sweepwholelist(L, &g->rootgc);
	for (i = 0; i < g->strt.size; i++)  /* free all string lists */
		sweepwholelist(L, &g->strt.hash[i]);
}


static void markmt (global_State *g) {
	int i;
	for (i=0; i<NUM_TAGS; i++)
		if (g->mt[i]) markobject(g, g->mt[i]);
}


/* mark root set */
static void markroot (lua_State *L) {
	global_State *g = G(L);
	g->gray = NULL;
	g->grayagain = NULL;
	g->weak = NULL;
	markobject(g, g->mainthread);
	/* make global table be traversed before main stack */
	markvalue(g, gt(g->mainthread));
	markvalue(g, registry(L));
	markmt(g);
	g->gcstate = GCSpropagate;
}


static void remarkupvals (global_State *g) {
	UpVal *uv;
	for (uv = g->uvhead.u.l.next; uv != &g->uvhead; uv = uv->u.l.next) {
		lua_assert(uv->u.l.next->u.l.prev == uv && uv->u.l.prev->u.l.next == uv);
		if (isgray(obj2gco(uv)))
			markvalue(g, uv->v);
	}
}


static void atomic (lua_State *L) {
	global_State *g = G(L);
	size_t udsize;  /* total size of userdata to be finalized */
	/* remark occasional upvalues of (maybe) dead threads */
	remarkupvals(g);
	/* traverse objects cautch by write barrier and by 'remarkupvals' */
	propagateall(g);
	/* remark weak tables */
	g->gray = g->weak;
	g->weak = NULL;
	lua_assert(!iswhite(obj2gco(g->mainthread)));
	markobject(g, L);  /* mark running thread */
	markmt(g);  /* mark basic metatables (again) */
	propagateall(g);
	/* remark gray again */
	g->gray = g->grayagain;
	g->grayagain = NULL;
	propagateall(g);
	udsize = luaC_separateudata(L, 0);  /* separate userdata to be finalized */
	marktmu(g);  /* mark `preserved' userdata */
	udsize += propagateall(g);  /* remark, to propagate `preserveness' */
	cleartable(g->weak);  /* remove collected objects from weak tables */
	/* flip current white */
	g->currentwhite = cast_byte(otherwhite(g));
	g->sweepstrgc = 0;
	g->sweepgc = &g->rootgc;
	g->gcstate = GCSsweepstring;
	g->estimate = g->totalbytes - udsize;  /* first estimate */
}


static l_mem singlestep (lua_State *L) {
	global_State *g = G(L);
	/*lua_checkmemory(L);*/
	switch (g->gcstate) {
	case GCSpause: {
		markroot(L);  /* start a new collection */
		return 0;
				   }
	case GCSpropagate: {
		if (g->gray)
			return propagatemark(g);
		else {  /* no more `gray' objects */
			atomic(L);  /* finish mark phase */
			return 0;
		}
					   }
	case GCSsweepstring: {
		lu_mem old = g->totalbytes;
		sweepwholelist(L, &g->strt.hash[g->sweepstrgc++]);
		if (g->sweepstrgc >= g->strt.size)  /* nothing more to sweep? */
			g->gcstate = GCSsweep;  /* end sweep-string phase */
		lua_assert(old >= g->totalbytes);
		g->estimate -= old - g->totalbytes;
		return GCSWEEPCOST;
						 }
	case GCSsweep: {
		lu_mem old = g->totalbytes;
		g->sweepgc = sweeplist(L, g->sweepgc, GCSWEEPMAX);
		if (*g->sweepgc == NULL) {  /* nothing more to sweep? */
			checkSizes(L);
			g->gcstate = GCSfinalize;  /* end sweep phase */
		}
		lua_assert(old >= g->totalbytes);
		g->estimate -= old - g->totalbytes;
		return GCSWEEPMAX*GCSWEEPCOST;
				   }
	case GCSfinalize: {
		if (g->tmudata) {
			GCTM(L);
			if (g->estimate > GCFINALIZECOST)
				g->estimate -= GCFINALIZECOST;
			return GCFINALIZECOST;
		}
		else {
			g->gcstate = GCSpause;  /* end collection */
			g->gcdept = 0;
			return 0;
		}
					  }
	default: lua_assert(0); return 0;
	}
}


void luaC_step (lua_State *L) {
	global_State *g = G(L);
	l_mem lim = (GCSTEPSIZE/100) * g->gcstepmul;
	if (lim == 0)
		lim = (MAX_LUMEM-1)/2;  /* no limit */
	g->gcdept += g->totalbytes - g->GCthreshold;
	do {
		lim -= singlestep(L);
		if (g->gcstate == GCSpause)
			break;
	} while (lim > 0);
	if (g->gcstate != GCSpause) {
		if (g->gcdept < GCSTEPSIZE)
			g->GCthreshold = g->totalbytes + GCSTEPSIZE;  /* - lim/g->gcstepmul;*/
		else {
			g->gcdept -= GCSTEPSIZE;
			g->GCthreshold = g->totalbytes;
		}
	}
	else {
		lua_assert(g->totalbytes >= g->estimate);
		setthreshold(g);
	}
}


void luaC_fullgc (lua_State *L) {
	global_State *g = G(L);
	if (g->gcstate <= GCSpropagate) {
		/* reset sweep marks to sweep all elements (returning them to white) */
		g->sweepstrgc = 0;
		g->sweepgc = &g->rootgc;
		/* reset other collector lists */
		g->gray = NULL;
		g->grayagain = NULL;
		g->weak = NULL;
		g->gcstate = GCSsweepstring;
	}
	lua_assert(g->gcstate != GCSpause && g->gcstate != GCSpropagate);
	/* finish any pending sweep phase */
	while (g->gcstate != GCSfinalize) {
		lua_assert(g->gcstate == GCSsweepstring || g->gcstate == GCSsweep);
		singlestep(L);
	}
	markroot(L);
	while (g->gcstate != GCSpause) {
		singlestep(L);
	}
	setthreshold(g);
}


void luaC_barrierf (lua_State *L, GCObject *o, GCObject *v) {
	global_State *g = G(L);
	lua_assert(isblack(o) && iswhite(v) && !isdead(g, v) && !isdead(g, o));
	lua_assert(g->gcstate != GCSfinalize && g->gcstate != GCSpause);
	lua_assert(ttype(&o->gch) != LUA_TTABLE);
	/* must keep invariant? */
	if (g->gcstate == GCSpropagate)
		reallymarkobject(g, v);  /* restore invariant */
	else  /* don't mind */
		makewhite(g, o);  /* mark as white just to avoid other barriers */
}


void luaC_barrierback (lua_State *L, Table *t) {
	global_State *g = G(L);
	GCObject *o = obj2gco(t);
	lua_assert(isblack(o) && !isdead(g, o));
	lua_assert(g->gcstate != GCSfinalize && g->gcstate != GCSpause);
	black2gray(o);  /* make table gray (again) */
	t->gclist = g->grayagain;
	g->grayagain = o;
}


void luaC_link (lua_State *L, GCObject *o, lu_byte tt) {
	global_State *g = G(L);
	o->gch.next = g->rootgc;
	g->rootgc = o;
	o->gch.marked = luaC_white(g);
	o->gch.tt = tt;
}


void luaC_linkupval (lua_State *L, UpVal *uv) {
	global_State *g = G(L);
	GCObject *o = obj2gco(uv);
	o->gch.next = g->rootgc;  /* link upvalue into `rootgc' list */
	g->rootgc = o;
	if (isgray(o)) { 
		if (g->gcstate == GCSpropagate) {
			gray2black(o);  /* closed upvalues need barrier */
			luaC_barrier(L, uv, uv->v);
		}
		else {  /* sweep phase: sweep it (turning it into white) */
			makewhite(g, o);
			lua_assert(g->gcstate != GCSfinalize && g->gcstate != GCSpause);
		}
	}
}



//-------------------------------------------------------------lundump.c------------------------------------------------------
typedef struct {
	lua_State* L;
	ZIO* Z;
	Mbuffer* b;
	const char* name;
} LoadState;

#ifdef LUAC_TRUST_BINARIES
#define IF(c,s)
#else
#define IF(c,s)		if (c) error(S,s)

static void error(LoadState* S, const char* why)
{
	luaO_pushfstring(S->L,"%s: %s in precompiled chunk",S->name,why);
	luaD_throw(S->L,LUA_ERRSYNTAX);
}
#endif

#define LoadMem(S,b,n,size)	LoadBlock(S,b,(n)*(size))
#define	LoadByte(S)		(lu_byte)LoadChar(S)
#define LoadVar(S,x)		LoadMem(S,&x,1,sizeof(x))
#define LoadVector(S,b,n,size)	LoadMem(S,b,n,size)

static void LoadBlock(LoadState* S, void* b, size_t size)
{
	size_t r=luaZ_read(S->Z,b,size);
	IF (r!=0, "unexpected end");
}

static int LoadChar(LoadState* S)
{
	char x;
	LoadVar(S,x);
	return x;
}

static int LoadInt(LoadState* S)
{
	int x;
	LoadVar(S,x);
	IF (x<0, "bad integer");
	return x;
}

static lua_Number LoadNumber(LoadState* S)
{
	lua_Number x;
	LoadVar(S,x);
	return x;
}

static TString* LoadString(LoadState* S)
{
	size_t size;
	LoadVar(S,size);
	if (size==0)
		return NULL;
	else
	{
		char* s=luaZ_openspace(S->L,S->b,size);
		LoadBlock(S,s,size);
		return luaS_newlstr(S->L,s,size-1);		/* remove trailing '\0' */
	}
}

static void LoadCode(LoadState* S, Proto* f)
{
	int n=LoadInt(S);
	f->code=luaM_newvector(S->L,n,Instruction);
	f->sizecode=n;
	LoadVector(S,f->code,n,sizeof(Instruction));
}

static Proto* LoadFunction(LoadState* S, TString* p);

static void LoadConstants(LoadState* S, Proto* f)
{
	int i,n;
	n=LoadInt(S);
	f->k=luaM_newvector(S->L,n,TValue);
	f->sizek=n;
	for (i=0; i<n; i++) setnilvalue(&f->k[i]);
	for (i=0; i<n; i++)
	{
		TValue* o=&f->k[i];
		int t=LoadChar(S);
		switch (t)
		{
		case LUA_TNIL:
			setnilvalue(o);
			break;
		case LUA_TBOOLEAN:
			setbvalue(o,LoadChar(S));
			break;
		case LUA_TNUMBER:
			setnvalue(o,LoadNumber(S));
			break;
		case LUA_TSTRING:
			setsvalue2n(S->L,o,LoadString(S));
			break;
		default:
			IF (1, "bad constant");
			break;
		}
	}
	n=LoadInt(S);
	f->p=luaM_newvector(S->L,n,Proto*);
	f->sizep=n;
	for (i=0; i<n; i++) f->p[i]=NULL;
	for (i=0; i<n; i++) f->p[i]=LoadFunction(S,f->source);
}

static void LoadDebug(LoadState* S, Proto* f)
{
	int i,n;
	n=LoadInt(S);
	f->lineinfo=luaM_newvector(S->L,n,int);
	f->sizelineinfo=n;
	LoadVector(S,f->lineinfo,n,sizeof(int));
	n=LoadInt(S);
	f->locvars=luaM_newvector(S->L,n,LocVar);
	f->sizelocvars=n;
	for (i=0; i<n; i++) f->locvars[i].varname=NULL;
	for (i=0; i<n; i++)
	{
		f->locvars[i].varname=LoadString(S);
		f->locvars[i].startpc=LoadInt(S);
		f->locvars[i].endpc=LoadInt(S);
	}
	n=LoadInt(S);
	f->upvalues=luaM_newvector(S->L,n,TString*);
	f->sizeupvalues=n;
	for (i=0; i<n; i++) f->upvalues[i]=NULL;
	for (i=0; i<n; i++) f->upvalues[i]=LoadString(S);
}

static Proto* LoadFunction(LoadState* S, TString* p)
{
	Proto* f=luaF_newproto(S->L);
	setptvalue2s(S->L,S->L->top,f); incr_top(S->L);
	f->source=LoadString(S); if (f->source==NULL) f->source=p;
	f->linedefined=LoadInt(S);
	f->lastlinedefined=LoadInt(S);
	f->nups=LoadByte(S);
	f->numparams=LoadByte(S);
	f->is_vararg=LoadByte(S);
	f->maxstacksize=LoadByte(S);
	LoadCode(S,f);
	LoadConstants(S,f);
	LoadDebug(S,f);
	IF (!luaG_checkcode(f), "bad code");
	S->L->top--;
	return f;
}

/*
* make header
*/
void luaU_header (char* h)
{
	int x=1;
	memcpy(h,LUA_SIGNATURE,sizeof(LUA_SIGNATURE)-1);
	h+=sizeof(LUA_SIGNATURE)-1;
	*h++=(char)LUAC_VERSION;
	*h++=(char)LUAC_FORMAT;
	*h++=(char)*(char*)&x;				/* endianness */
	*h++=(char)sizeof(int);
	*h++=(char)sizeof(size_t);
	*h++=(char)sizeof(Instruction);
	*h++=(char)sizeof(lua_Number);
	*h++=(char)(((lua_Number)0.5)==0);		/* is lua_Number integral? */
}

static void LoadHeader(LoadState* S)
{
	char h[LUAC_HEADERSIZE];
	char s[LUAC_HEADERSIZE];
	luaU_header(h);
	LoadBlock(S,s,LUAC_HEADERSIZE);
	IF (memcmp(h,s,LUAC_HEADERSIZE)!=0, "bad header");
}

/*
** load precompiled chunk
*/
Proto* luaU_undump (lua_State* L, ZIO* Z, Mbuffer* buff, const char* name)
{
	LoadState S;
	if (*name=='@' || *name=='=')
		S.name=name+1;
	else if (*name==LUA_SIGNATURE[0])
		S.name="binary string";
	else
		S.name=name;
	S.L=L;
	S.Z=Z;
	S.b=buff;
	LoadHeader(&S);
	return LoadFunction(&S,luaS_newliteral(L,"=?"));
}

//-------------------------------------------------------------ldump.c------------------------------------------------------
typedef struct {
	lua_State* L;
	lua_Writer writer;
	void* data;
	int strip;
	int status;
} DumpState;

#define DumpMem(b,n,size,D)	DumpBlock(b,(n)*(size),D)
#define DumpVar(x,D)	 	DumpMem(&x,1,sizeof(x),D)

static void DumpBlock(const void* b, size_t size, DumpState* D)
{
	if (D->status==0)
	{
		lua_unlock(D->L);
		D->status=(*D->writer)(D->L,b,size,D->data);
		lua_lock(D->L);
	}
}

static void DumpChar(int y, DumpState* D)
{
	char x=(char)y;
	DumpVar(x,D);
}

static void DumpInt(int x, DumpState* D)
{
	DumpVar(x,D);
}

static void DumpNumber(lua_Number x, DumpState* D)
{
	DumpVar(x,D);
}

static void DumpVector(const void* b, int n, size_t size, DumpState* D)
{
	DumpInt(n,D);
	DumpMem(b,n,size,D);
}

static void DumpString(const TString* s, DumpState* D)
{
	if (s==NULL || getstr(s)==NULL)
	{
		size_t size=0;
		DumpVar(size,D);
	}
	else
	{
		size_t size=s->tsv.len+1;		/* include trailing '\0' */
		DumpVar(size,D);
		DumpBlock(getstr(s),size,D);
	}
}

#define DumpCode(f,D)	 DumpVector(f->code,f->sizecode,sizeof(Instruction),D)

static void DumpFunction(const Proto* f, const TString* p, DumpState* D);

static void DumpConstants(const Proto* f, DumpState* D)
{
	int i,n=f->sizek;
	DumpInt(n,D);
	for (i=0; i<n; i++)
	{
		const TValue* o=&f->k[i];
		DumpChar(ttype(o),D);
		switch (ttype(o))
		{
		case LUA_TNIL:
			break;
		case LUA_TBOOLEAN:
			DumpChar(bvalue(o),D);
			break;
		case LUA_TNUMBER:
			DumpNumber(nvalue(o),D);
			break;
		case LUA_TSTRING:
			DumpString(rawtsvalue(o),D);
			break;
		default:
			lua_assert(0);			/* cannot happen */
			break;
		}
	}
	n=f->sizep;
	DumpInt(n,D);
	for (i=0; i<n; i++) DumpFunction(f->p[i],f->source,D);
}

static void DumpDebug(const Proto* f, DumpState* D)
{
	int i,n;
	n= (D->strip) ? 0 : f->sizelineinfo;
	DumpVector(f->lineinfo,n,sizeof(int),D);
	n= (D->strip) ? 0 : f->sizelocvars;
	DumpInt(n,D);
	for (i=0; i<n; i++)
	{
		DumpString(f->locvars[i].varname,D);
		DumpInt(f->locvars[i].startpc,D);
		DumpInt(f->locvars[i].endpc,D);
	}
	n= (D->strip) ? 0 : f->sizeupvalues;
	DumpInt(n,D);
	for (i=0; i<n; i++) DumpString(f->upvalues[i],D);
}

static void DumpFunction(const Proto* f, const TString* p, DumpState* D)
{
	DumpString((f->source==p || D->strip) ? NULL : f->source,D);
	DumpInt(f->linedefined,D);
	DumpInt(f->lastlinedefined,D);
	DumpChar(f->nups,D);
	DumpChar(f->numparams,D);
	DumpChar(f->is_vararg,D);
	DumpChar(f->maxstacksize,D);
	DumpCode(f,D);
	DumpConstants(f,D);
	DumpDebug(f,D);
}

static void DumpHeader(DumpState* D)
{
	char h[LUAC_HEADERSIZE];
	luaU_header(h);
	DumpBlock(h,LUAC_HEADERSIZE,D);
}

/*
** dump Lua function as precompiled chunk
*/
int luaU_dump (lua_State* L, const Proto* f, lua_Writer w, void* data, int strip)
{
	DumpState D;
	D.L=L;
	D.writer=w;
	D.data=data;
	D.strip=strip;
	D.status=0;
	DumpHeader(&D);
	DumpFunction(f,NULL,&D);
	return D.status;
}

//-------------------------------------------------------------lapi.c-------------------------------------------------------
const char lua_ident[] =
"$Lua: " LUA_RELEASE " " LUA_COPYRIGHT " $\n"
"$Authors: " LUA_AUTHORS " $\n"
"$URL: www.lua.org $\n";


#define api_checknelems(L, n)	api_check(L, (n) <= (L->top - L->base))
#define api_checkvalidindex(L, i)	api_check(L, (i) != luaO_nilobject)
#define api_incr_top(L)   {api_check(L, L->top < L->ci->top); L->top++;}

/*
** Union of all collectable objects
*/
typedef union GCObject GCObject;

/*
** Union of all Lua values
*/
//typedef union {
//  GCObject *gc;
//  void *p;
//  lua_Number n;
//  int b;
//} Value;

/*
** $Id: lapi.c,v 2.55 2006/06/07 12:37:17 roberto Exp $
** Lua API
** See Copyright Notice in lua.h
*/


#define TValuefields	Value value; int tt

//typedef struct lua_TValue {
//	TValuefields;
//} TValue;

#ifndef cast
#define cast(t, exp)	((t)(exp))
#endif




/*
** Common Header for all collectable objects (in macro form, to be
** included in other objects)
*/
#define CommonHeader	GCObject *next; lu_byte tt; lu_byte marked

static TValue *index2adr (lua_State *L, int idx) {
	if (idx > 0) {
		TValue *o = L->base + (idx - 1);
		api_check(L, idx <= L->ci->top - L->base);
		if (o >= L->top) return cast(TValue *, luaO_nilobject);
		else return o;
	}
	else if (idx > LUA_REGISTRYINDEX) {
		api_check(L, idx != 0 && -idx <= L->top - L->base);
		return L->top + idx;
	}
	else switch (idx) {  /* pseudo-indices */
	case LUA_REGISTRYINDEX: return registry(L);
	case LUA_ENVIRONINDEX: {
		Closure *func = curr_func(L);
		sethvalue(L, &L->env, func->c.env);
		return &L->env;
						   }
	case LUA_GLOBALSINDEX: return gt(L);
	default: {
		Closure *func = curr_func(L);
		idx = LUA_GLOBALSINDEX - idx;
		return (idx <= func->c.nupvalues)
			? &func->c.upvalue[idx-1]
		: cast(TValue *, luaO_nilobject);
			 }
	}
}


static Table *getcurrenv (lua_State *L) {
	if (L->ci == L->base_ci)  /* no enclosing function? */
		return hvalue(gt(L));  /* use global table as environment */
	else {
		Closure *func = curr_func(L);
		return func->c.env;
	}
}


void luaA_pushobject (lua_State *L, const TValue *o) {
	setobj2s(L, L->top, o);
	api_incr_top(L);
}


LUA_API int lua_checkstack (lua_State *L, int size) {
	int res;
	lua_lock(L);
	if ((L->top - L->base + size) > LUAI_MAXCSTACK)
		res = 0;  /* stack overflow */
	else {
		luaD_checkstack(L, size);
		if (L->ci->top < L->top + size)
			L->ci->top = L->top + size;
		res = 1;
	}
	lua_unlock(L);
	return res;
}


LUA_API void lua_xmove (lua_State *from, lua_State *to, int n) {
	int i;
	if (from == to) return;
	lua_lock(to);
	api_checknelems(from, n);
	api_check(from, G(from) == G(to));
	api_check(from, to->ci->top - to->top >= n);
	from->top -= n;
	for (i = 0; i < n; i++) {
		setobj2s(to, to->top++, from->top + i);
	}
	lua_unlock(to);
}


LUA_API lua_CFunction lua_atpanic (lua_State *L, lua_CFunction panicf) {
	lua_CFunction old;
	lua_lock(L);
	old = G(L)->panic;
	G(L)->panic = panicf;
	lua_unlock(L);
	return old;
}


LUA_API lua_State *lua_newthread (lua_State *L) {
	lua_State *L1;
	lua_lock(L);
	luaC_checkGC(L);
	L1 = luaE_newthread(L);
	setthvalue(L, L->top, L1);
	api_incr_top(L);
	lua_unlock(L);
	luai_userstatethread(L, L1);
	return L1;
}



/*
** basic stack manipulation
*/


LUA_API int lua_gettop (lua_State *L) {
	return cast_int(L->top - L->base);
}


LUA_API void lua_settop (lua_State *L, int idx) {
	lua_lock(L);
	if (idx >= 0) {
		api_check(L, idx <= L->stack_last - L->base);
		while (L->top < L->base + idx)
			setnilvalue(L->top++);
		L->top = L->base + idx;
	}
	else {
		api_check(L, -(idx+1) <= (L->top - L->base));
		L->top += idx+1;  /* `subtract' index (index is negative) */
	}
	lua_unlock(L);
}


LUA_API void lua_remove (lua_State *L, int idx) {
	StkId p;
	lua_lock(L);
	p = index2adr(L, idx);
	api_checkvalidindex(L, p);
	while (++p < L->top) setobjs2s(L, p-1, p);
	L->top--;
	lua_unlock(L);
}


LUA_API void lua_insert (lua_State *L, int idx) {
	StkId p;
	StkId q;
	lua_lock(L);
	p = index2adr(L, idx);
	api_checkvalidindex(L, p);
	for (q = L->top; q>p; q--) setobjs2s(L, q, q-1);
	setobjs2s(L, p, L->top);
	lua_unlock(L);
}


LUA_API void lua_replace (lua_State *L, int idx) {
	StkId o;
	lua_lock(L);
	/* explicit test for incompatible code */
	if (idx == LUA_ENVIRONINDEX && L->ci == L->base_ci)
		luaG_runerror(L, "no calling environment");
	api_checknelems(L, 1);
	o = index2adr(L, idx);
	api_checkvalidindex(L, o);
	if (idx == LUA_ENVIRONINDEX) {
		Closure *func = curr_func(L);
		api_check(L, ttistable(L->top - 1)); 
		func->c.env = hvalue(L->top - 1);
		luaC_barrier(L, func, L->top - 1);
	}
	else {
		setobj(L, o, L->top - 1);
		if (idx < LUA_GLOBALSINDEX)  /* function upvalue? */
			luaC_barrier(L, curr_func(L), L->top - 1);
	}
	L->top--;
	lua_unlock(L);
}


LUA_API void lua_pushvalue (lua_State *L, int idx) {
	lua_lock(L);
	setobj2s(L, L->top, index2adr(L, idx));
	api_incr_top(L);
	lua_unlock(L);
}



/*
** access functions (stack -> C)
*/


LUA_API int lua_type (lua_State *L, int idx) {
	StkId o = index2adr(L, idx);
	return (o == luaO_nilobject) ? LUA_TNONE : ttype(o);
}


LUA_API const char *lua_typename (lua_State *L, int t) {
	UNUSED(L);
	return (t == LUA_TNONE) ? "no value" : luaT_typenames[t];
}


LUA_API int lua_iscfunction (lua_State *L, int idx) {
	StkId o = index2adr(L, idx);
	return iscfunction(o);
}


LUA_API int lua_isnumber (lua_State *L, int idx) {
	TValue n;
	const TValue *o = index2adr(L, idx);
	return tonumber(o, &n);
}


LUA_API int lua_isstring (lua_State *L, int idx) {
	int t = lua_type(L, idx);
	return (t == LUA_TSTRING || t == LUA_TNUMBER);
}


LUA_API int lua_isuserdata (lua_State *L, int idx) {
	const TValue *o = index2adr(L, idx);
	return (ttisuserdata(o) || ttislightuserdata(o));
}


LUA_API int lua_rawequal (lua_State *L, int index1, int index2) {
	StkId o1 = index2adr(L, index1);
	StkId o2 = index2adr(L, index2);
	return (o1 == luaO_nilobject || o2 == luaO_nilobject) ? 0
		: luaO_rawequalObj(o1, o2);
}


LUA_API int lua_equal (lua_State *L, int index1, int index2) {
	StkId o1, o2;
	int i;
	lua_lock(L);  /* may call tag method */
	o1 = index2adr(L, index1);
	o2 = index2adr(L, index2);
	i = (o1 == luaO_nilobject || o2 == luaO_nilobject) ? 0 : equalobj(L, o1, o2);
	lua_unlock(L);
	return i;
}


LUA_API int lua_lessthan (lua_State *L, int index1, int index2) {
	StkId o1, o2;
	int i;
	lua_lock(L);  /* may call tag method */
	o1 = index2adr(L, index1);
	o2 = index2adr(L, index2);
	i = (o1 == luaO_nilobject || o2 == luaO_nilobject) ? 0
		: luaV_lessthan(L, o1, o2);
	lua_unlock(L);
	return i;
}



LUA_API lua_Number lua_tonumber (lua_State *L, int idx) {
	TValue n;
	const TValue *o = index2adr(L, idx);
	if (tonumber(o, &n))
		return nvalue(o);
	else
		return 0;
}


LUA_API lua_Integer lua_tointeger (lua_State *L, int idx) {
	TValue n;
	const TValue *o = index2adr(L, idx);
	if (tonumber(o, &n)) {
		lua_Integer res;
		lua_Number num = nvalue(o);
		lua_number2integer(res, num);
		return res;
	}
	else
		return 0;
}


LUA_API int lua_toboolean (lua_State *L, int idx) {
	const TValue *o = index2adr(L, idx);
	return !l_isfalse(o);
}


LUA_API const char *lua_tolstring (lua_State *L, int idx, size_t *len) {
	StkId o = index2adr(L, idx);
	if (!ttisstring(o)) {
		lua_lock(L);  /* `luaV_tostring' may create a new string */
		if (!luaV_tostring(L, o)) {  /* conversion failed? */
			if (len != NULL) *len = 0;
			lua_unlock(L);
			return NULL;
		}
		luaC_checkGC(L);
		o = index2adr(L, idx);  /* previous call may reallocate the stack */
		lua_unlock(L);
	}
	if (len != NULL) *len = tsvalue(o)->len;
	return svalue(o);
}


LUA_API size_t lua_objlen (lua_State *L, int idx) {
	StkId o = index2adr(L, idx);
	switch (ttype(o)) {
	case LUA_TSTRING: return tsvalue(o)->len;
	case LUA_TUSERDATA: return uvalue(o)->len;
	case LUA_TTABLE: return luaH_getn(hvalue(o));
	case LUA_TNUMBER: {
		size_t l;
		lua_lock(L);  /* `luaV_tostring' may create a new string */
		l = (luaV_tostring(L, o) ? tsvalue(o)->len : 0);
		lua_unlock(L);
		return l;
					  }
	default: return 0;
	}
}


LUA_API lua_CFunction lua_tocfunction (lua_State *L, int idx) {
	StkId o = index2adr(L, idx);
	return (!iscfunction(o)) ? NULL : clvalue(o)->c.f;
}


LUA_API void *lua_touserdata (lua_State *L, int idx) {
	StkId o = index2adr(L, idx);
	switch (ttype(o)) {
	case LUA_TUSERDATA: return (rawuvalue(o) + 1);
	case LUA_TLIGHTUSERDATA: return pvalue(o);
	default: return NULL;
	}
}


LUA_API lua_State *lua_tothread (lua_State *L, int idx) {
	StkId o = index2adr(L, idx);
	return (!ttisthread(o)) ? NULL : thvalue(o);
}


LUA_API const void *lua_topointer (lua_State *L, int idx) {
	StkId o = index2adr(L, idx);
	switch (ttype(o)) {
	case LUA_TTABLE: return hvalue(o);
	case LUA_TFUNCTION: return clvalue(o);
	case LUA_TTHREAD: return thvalue(o);
	case LUA_TUSERDATA:
	case LUA_TLIGHTUSERDATA:
		return lua_touserdata(L, idx);
	default: return NULL;
	}
}



/*
** push functions (C -> stack)
*/


LUA_API void lua_pushnil (lua_State *L) {
	lua_lock(L);
	setnilvalue(L->top);
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API void lua_pushnumber (lua_State *L, lua_Number n) {
	lua_lock(L);
	setnvalue(L->top, n);
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API void lua_pushinteger (lua_State *L, lua_Integer n) {
	lua_lock(L);
	setnvalue(L->top, cast_num(n));
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API void lua_pushlstring (lua_State *L, const char *s, size_t len) {
	lua_lock(L);
	luaC_checkGC(L);
	setsvalue2s(L, L->top, luaS_newlstr(L, s, len));
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API void lua_pushstring (lua_State *L, const char *s) {
	if (s == NULL)
		lua_pushnil(L);
	else
		lua_pushlstring(L, s, strlen(s));
}


LUA_API const char *lua_pushvfstring (lua_State *L, const char *fmt,
									  va_list argp) {
										  const char *ret;
										  lua_lock(L);
										  luaC_checkGC(L);
										  ret = luaO_pushvfstring(L, fmt, argp);
										  lua_unlock(L);
										  return ret;
}


LUA_API const char *lua_pushfstring (lua_State *L, const char *fmt, ...) {
	const char *ret;
	va_list argp;
	lua_lock(L);
	luaC_checkGC(L);
	va_start(argp, fmt);
	ret = luaO_pushvfstring(L, fmt, argp);
	va_end(argp);
	lua_unlock(L);
	return ret;
}


LUA_API void lua_pushcclosure (lua_State *L, lua_CFunction fn, int n) {
	Closure *cl;
	lua_lock(L);
	luaC_checkGC(L);
	api_checknelems(L, n);
	cl = luaF_newCclosure(L, n, getcurrenv(L));
	cl->c.f = fn;
	L->top -= n;
	while (n--)
		setobj2n(L, &cl->c.upvalue[n], L->top+n);
	setclvalue(L, L->top, cl);
	lua_assert(iswhite(obj2gco(cl)));
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API void lua_pushboolean (lua_State *L, int b) {
	lua_lock(L);
	setbvalue(L->top, (b != 0));  /* ensure that true is 1 */
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API void lua_pushlightuserdata (lua_State *L, void *p) {
	lua_lock(L);
	setpvalue(L->top, p);
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API int lua_pushthread (lua_State *L) {
	lua_lock(L);
	setthvalue(L, L->top, L);
	api_incr_top(L);
	lua_unlock(L);
	return (G(L)->mainthread == L);
}



/*
** get functions (Lua -> stack)
*/


LUA_API void lua_gettable (lua_State *L, int idx) {
	StkId t;
	lua_lock(L);
	t = index2adr(L, idx);
	api_checkvalidindex(L, t);
	luaV_gettable(L, t, L->top - 1, L->top - 1);
	lua_unlock(L);
}


LUA_API void lua_getfield (lua_State *L, int idx, const char *k) {
	StkId t;
	TValue key;
	lua_lock(L);
	t = index2adr(L, idx);
	api_checkvalidindex(L, t);
	setsvalue(L, &key, luaS_new(L, k));
	luaV_gettable(L, t, &key, L->top);
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API void lua_rawget (lua_State *L, int idx) {
	StkId t;
	lua_lock(L);
	t = index2adr(L, idx);
	api_check(L, ttistable(t));
	setobj2s(L, L->top - 1, luaH_get(hvalue(t), L->top - 1));
	lua_unlock(L);
}


LUA_API void lua_rawgeti (lua_State *L, int idx, int n) {
	StkId o;
	lua_lock(L);
	o = index2adr(L, idx);
	api_check(L, ttistable(o));
	setobj2s(L, L->top, luaH_getnum(hvalue(o), n));
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API void lua_createtable (lua_State *L, int narray, int nrec) {
	lua_lock(L);
	luaC_checkGC(L);
	sethvalue(L, L->top, luaH_new(L, narray, nrec));
	api_incr_top(L);
	lua_unlock(L);
}


LUA_API int lua_getmetatable (lua_State *L, int objindex) {
	const TValue *obj;
	Table *mt = NULL;
	int res;
	lua_lock(L);
	obj = index2adr(L, objindex);
	switch (ttype(obj)) {
	case LUA_TTABLE:
		mt = hvalue(obj)->metatable;
		break;
	case LUA_TUSERDATA:
		mt = uvalue(obj)->metatable;
		break;
	default:
		mt = G(L)->mt[ttype(obj)];
		break;
	}
	if (mt == NULL)
		res = 0;
	else {
		sethvalue(L, L->top, mt);
		api_incr_top(L);
		res = 1;
	}
	lua_unlock(L);
	return res;
}


LUA_API void lua_getfenv (lua_State *L, int idx) {
	StkId o;
	lua_lock(L);
	o = index2adr(L, idx);
	api_checkvalidindex(L, o);
	switch (ttype(o)) {
	case LUA_TFUNCTION:
		sethvalue(L, L->top, clvalue(o)->c.env);
		break;
	case LUA_TUSERDATA:
		sethvalue(L, L->top, uvalue(o)->env);
		break;
	case LUA_TTHREAD:
		setobj2s(L, L->top,  gt(thvalue(o)));
		break;
	default:
		setnilvalue(L->top);
		break;
	}
	api_incr_top(L);
	lua_unlock(L);
}


/*
** set functions (stack -> Lua)
*/


LUA_API void lua_settable (lua_State *L, int idx) {
	StkId t;
	lua_lock(L);
	api_checknelems(L, 2);
	t = index2adr(L, idx);
	api_checkvalidindex(L, t);
	luaV_settable(L, t, L->top - 2, L->top - 1);
	L->top -= 2;  /* pop index and value */
	lua_unlock(L);
}


LUA_API void lua_setfield (lua_State *L, int idx, const char *k) {
	StkId t;
	TValue key;
	lua_lock(L);
	api_checknelems(L, 1);
	t = index2adr(L, idx);
	api_checkvalidindex(L, t);
	setsvalue(L, &key, luaS_new(L, k));
	luaV_settable(L, t, &key, L->top - 1);
	L->top--;  /* pop value */
	lua_unlock(L);
}


LUA_API void lua_rawset (lua_State *L, int idx) {
	StkId t;
	lua_lock(L);
	api_checknelems(L, 2);
	t = index2adr(L, idx);
	api_check(L, ttistable(t));
	setobj2t(L, luaH_set(L, hvalue(t), L->top-2), L->top-1);
	luaC_barriert(L, hvalue(t), L->top-1);
	L->top -= 2;
	lua_unlock(L);
}


LUA_API void lua_rawseti (lua_State *L, int idx, int n) {
	StkId o;
	lua_lock(L);
	api_checknelems(L, 1);
	o = index2adr(L, idx);
	api_check(L, ttistable(o));
	setobj2t(L, luaH_setnum(L, hvalue(o), n), L->top-1);
	luaC_barriert(L, hvalue(o), L->top-1);
	L->top--;
	lua_unlock(L);
}


LUA_API int lua_setmetatable (lua_State *L, int objindex) {
	TValue *obj;
	Table *mt;
	lua_lock(L);
	api_checknelems(L, 1);
	obj = index2adr(L, objindex);
	api_checkvalidindex(L, obj);
	if (ttisnil(L->top - 1))
		mt = NULL;
	else {
		api_check(L, ttistable(L->top - 1));
		mt = hvalue(L->top - 1);
	}
	switch (ttype(obj)) {
	case LUA_TTABLE: {
		hvalue(obj)->metatable = mt;
		if (mt)
			luaC_objbarriert(L, hvalue(obj), mt);
		break;
					 }
	case LUA_TUSERDATA: {
		uvalue(obj)->metatable = mt;
		if (mt)
			luaC_objbarrier(L, rawuvalue(obj), mt);
		break;
						}
	default: {
		G(L)->mt[ttype(obj)] = mt;
		break;
			 }
	}
	L->top--;
	lua_unlock(L);
	return 1;
}


LUA_API int lua_setfenv (lua_State *L, int idx) {
	StkId o;
	int res = 1;
	lua_lock(L);
	api_checknelems(L, 1);
	o = index2adr(L, idx);
	api_checkvalidindex(L, o);
	api_check(L, ttistable(L->top - 1));
	switch (ttype(o)) {
	case LUA_TFUNCTION:
		clvalue(o)->c.env = hvalue(L->top - 1);
		break;
	case LUA_TUSERDATA:
		uvalue(o)->env = hvalue(L->top - 1);
		break;
	case LUA_TTHREAD:
		sethvalue(L, gt(thvalue(o)), hvalue(L->top - 1));
		break;
	default:
		res = 0;
		break;
	}
	luaC_objbarrier(L, gcvalue(o), hvalue(L->top - 1));
	L->top--;
	lua_unlock(L);
	return res;
}


/*
** `load' and `call' functions (run Lua code)
*/


#define adjustresults(L,nres) \
{ if (nres == LUA_MULTRET && L->top >= L->ci->top) L->ci->top = L->top; }


#define checkresults(L,na,nr) \
	api_check(L, (nr) == LUA_MULTRET || (L->ci->top - L->top >= (nr) - (na)))


LUA_API void lua_call (lua_State *L, int nargs, int nresults) {
	StkId func;
	lua_lock(L);
	api_checknelems(L, nargs+1);
	checkresults(L, nargs, nresults);
	func = L->top - (nargs+1);
	luaD_call(L, func, nresults);
	adjustresults(L, nresults);
	lua_unlock(L);
}



/*
** Execute a protected call.
*/
struct CallS {  /* data to `f_call' */
	StkId func;
	int nresults;
};


static void f_call (lua_State *L, void *ud) {
	struct CallS *c = cast(struct CallS *, ud);
	luaD_call(L, c->func, c->nresults);
}



LUA_API int lua_pcall (lua_State *L, int nargs, int nresults, int errfunc) {
	struct CallS c;
	int status;
	ptrdiff_t func;
	lua_lock(L);
	api_checknelems(L, nargs+1);
	checkresults(L, nargs, nresults);
	if (errfunc == 0)
		func = 0;
	else {
		StkId o = index2adr(L, errfunc);
		api_checkvalidindex(L, o);
		func = savestack(L, o);
	}
	c.func = L->top - (nargs+1);  /* function to be called */
	c.nresults = nresults;
	status = luaD_pcall(L, f_call, &c, savestack(L, c.func), func);
	adjustresults(L, nresults);
	lua_unlock(L);
	return status;
}


/*
** Execute a protected C call.
*/
struct CCallS {  /* data to `f_Ccall' */
	lua_CFunction func;
	void *ud;
};


static void f_Ccall (lua_State *L, void *ud) {
	struct CCallS *c = cast(struct CCallS *, ud);
	Closure *cl;
	cl = luaF_newCclosure(L, 0, getcurrenv(L));
	cl->c.f = c->func;
	setclvalue(L, L->top, cl);  /* push function */
	api_incr_top(L);
	setpvalue(L->top, c->ud);  /* push only argument */
	api_incr_top(L);
	luaD_call(L, L->top - 2, 0);
}


LUA_API int lua_cpcall (lua_State *L, lua_CFunction func, void *ud) {
	struct CCallS c;
	int status;
	lua_lock(L);
	c.func = func;
	c.ud = ud;
	status = luaD_pcall(L, f_Ccall, &c, savestack(L, L->top), 0);
	lua_unlock(L);
	return status;
}


LUA_API int lua_load (lua_State *L, lua_Reader reader, void *data,
					  const char *chunkname) {
						  ZIO z;
						  int status;
						  lua_lock(L);
						  if (!chunkname) chunkname = "?";
						  luaZ_init(L, &z, reader, data);
						  status = luaD_protectedparser(L, &z, chunkname);
						  lua_unlock(L);
						  return status;
}


LUA_API int lua_dump (lua_State *L, lua_Writer writer, void *data) {
	int status;
	TValue *o;
	lua_lock(L);
	api_checknelems(L, 1);
	o = L->top - 1;
	if (isLfunction(o))
		status = luaU_dump(L, clvalue(o)->l.p, writer, data, 0);
	else
		status = 1;
	lua_unlock(L);
	return status;
}


LUA_API int  lua_status (lua_State *L) {
	return L->status;
}


/*
** Garbage-collection function
*/

LUA_API int lua_gc (lua_State *L, int what, int data) {
	int res = 0;
	global_State *g;
	lua_lock(L);
	g = G(L);
	switch (what) {
	case LUA_GCSTOP: {
		g->GCthreshold = MAX_LUMEM;
		break;
					 }
	case LUA_GCRESTART: {
		g->GCthreshold = g->totalbytes;
		break;
						}
	case LUA_GCCOLLECT: {
		luaC_fullgc(L);
		break;
						}
	case LUA_GCCOUNT: {
		/* GC values are expressed in Kbytes: #bytes/2^10 */
		res = cast_int(g->totalbytes >> 10);
		break;
					  }
	case LUA_GCCOUNTB: {
		res = cast_int(g->totalbytes & 0x3ff);
		break;
					   }
	case LUA_GCSTEP: {
		lu_mem a = (cast(lu_mem, data) << 10);
		if (a <= g->totalbytes)
			g->GCthreshold = g->totalbytes - a;
		else
			g->GCthreshold = 0;
		while (g->GCthreshold <= g->totalbytes)
			luaC_step(L);
		if (g->gcstate == GCSpause)  /* end of cycle? */
			res = 1;  /* signal it */
		break;
					 }
	case LUA_GCSETPAUSE: {
		res = g->gcpause;
		g->gcpause = data;
		break;
						 }
	case LUA_GCSETSTEPMUL: {
		res = g->gcstepmul;
		g->gcstepmul = data;
		break;
						   }
	default: res = -1;  /* invalid option */
	}
	lua_unlock(L);
	return res;
}



/*
** miscellaneous functions
*/


LUA_API int lua_error (lua_State *L) {
	lua_lock(L);
	api_checknelems(L, 1);
	luaG_errormsg(L);
	lua_unlock(L);
	return 0;  /* to avoid warnings */
}


LUA_API int lua_next (lua_State *L, int idx) {
	StkId t;
	int more;
	lua_lock(L);
	t = index2adr(L, idx);
	api_check(L, ttistable(t));
	more = luaH_next(L, hvalue(t), L->top - 1);
	if (more) {
		api_incr_top(L);
	}
	else  /* no more elements */
		L->top -= 1;  /* remove key */
	lua_unlock(L);
	return more;
}


LUA_API void lua_concat (lua_State *L, int n) {
	lua_lock(L);
	api_checknelems(L, n);
	if (n >= 2) {
		luaC_checkGC(L);
		luaV_concat(L, n, cast_int(L->top - L->base) - 1);
		L->top -= (n-1);
	}
	else if (n == 0) {  /* push empty string */
		setsvalue2s(L, L->top, luaS_newlstr(L, "", 0));
		api_incr_top(L);
	}
	/* else n == 1; nothing to do */
	lua_unlock(L);
}


LUA_API lua_Alloc lua_getallocf (lua_State *L, void **ud) {
	lua_Alloc f;
	lua_lock(L);
	if (ud) *ud = G(L)->ud;
	f = G(L)->frealloc;
	lua_unlock(L);
	return f;
}


LUA_API void lua_setallocf (lua_State *L, lua_Alloc f, void *ud) {
	lua_lock(L);
	G(L)->ud = ud;
	G(L)->frealloc = f;
	lua_unlock(L);
}


LUA_API void *lua_newuserdata (lua_State *L, size_t size) {
	Udata *u;
	lua_lock(L);
	luaC_checkGC(L);
	u = luaS_newudata(L, size, getcurrenv(L));
	setuvalue(L, L->top, u);
	api_incr_top(L);
	lua_unlock(L);
	return u + 1;
}




static const char *aux_upvalue (StkId fi, int n, TValue **val) {
	Closure *f;
	if (!ttisfunction(fi)) return NULL;
	f = clvalue(fi);
	if (f->c.isC) {
		if (!(1 <= n && n <= f->c.nupvalues)) return NULL;
		*val = &f->c.upvalue[n-1];
		return "";
	}
	else {
		Proto *p = f->l.p;
		if (!(1 <= n && n <= p->sizeupvalues)) return NULL;
		*val = f->l.upvals[n-1]->v;
		return getstr(p->upvalues[n-1]);
	}
}


LUA_API const char *lua_getupvalue (lua_State *L, int funcindex, int n) {
	const char *name;
	TValue *val;
	lua_lock(L);
	name = aux_upvalue(index2adr(L, funcindex), n, &val);
	if (name) {
		setobj2s(L, L->top, val);
		api_incr_top(L);
	}
	lua_unlock(L);
	return name;
}


LUA_API const char *lua_setupvalue (lua_State *L, int funcindex, int n) {
	const char *name;
	TValue *val;
	StkId fi;
	lua_lock(L);
	fi = index2adr(L, funcindex);
	api_checknelems(L, 1);
	name = aux_upvalue(fi, n, &val);
	if (name) {
		L->top--;
		setobj(L, val, L->top);
		luaC_barrier(L, clvalue(fi), L->top);
	}
	lua_unlock(L);
	return name;
}



//-------------------------------------------------------------lmem.c-------------------------------------------------------
/*
** About the realloc function:
** void * frealloc (void *ud, void *ptr, size_t osize, size_t nsize);
** (`osize' is the old size, `nsize' is the new size)
**
** Lua ensures that (ptr == NULL) iff (osize == 0).
**
** * frealloc(ud, NULL, 0, x) creates a new block of size `x'
**
** * frealloc(ud, p, x, 0) frees the block `p'
** (in this specific case, frealloc must return NULL).
** particularly, frealloc(ud, NULL, 0, 0) does nothing
** (which is equivalent to free(NULL) in ANSI C)
**
** frealloc returns NULL if it cannot create or reallocate the area
** (any reallocation to an equal or smaller size cannot fail!)
*/



#define MINSIZEARRAY	4

void *luaM_growaux_ (lua_State *L, void *block, int *size, size_t size_elems, int limit, const char *errormsg) {
						 void *newblock;
						 int newsize;
						 if (*size >= limit/2) {  /* cannot double it? */
							 if (*size >= limit)  /* cannot grow even a little? */
								 luaG_runerror(L, errormsg);
							 newsize = limit;  /* still have at least one free place */
						 }
						 else {
							 newsize = (*size)*2;
							 if (newsize < MINSIZEARRAY)
								 newsize = MINSIZEARRAY;  /* minimum size */
						 }
						 newblock = luaM_reallocv(L, block, *size, newsize, size_elems);
						 *size = newsize;  /* update only when everything else is OK */
						 return newblock;
}


void *luaM_toobig (lua_State *L) {
	luaG_runerror(L, "memory allocation error: block too big");
	return NULL;  /* to avoid warnings */
}



/*
** generic allocation routine.
*/
void *luaM_realloc_ (lua_State *L, void *block, size_t osize, size_t nsize) {
	global_State *g = G(L);
	lua_assert((osize == 0) == (block == NULL));
	block = (*g->frealloc)(g->ud, block, osize, nsize);
	if (block == NULL && nsize > 0)
		luaD_throw(L, LUA_ERRMEM);
	lua_assert((nsize == 0) == (block == NULL));
	g->totalbytes = (g->totalbytes - osize) + nsize;
	return block;
}

//-------------------------------------------------------------llex.c-------------------------------------------------------
LUAI_FUNC void luaX_lexerror (LexState *ls, const char *msg, int token);

#define next(ls) (ls->current = zgetc(ls->z))
#define currIsNewline(ls)	(ls->current == '\n' || ls->current == '\r')

/* ORDER RESERVED */
const char *const luaX_tokens [] = {
	"and", "break", "do", "else", "elseif",
	"end", "false", "for", "function", "if",
	"in", "local", "nil", "not", "or", "repeat",
	"return", "then", "true", "until", "while",
	"..", "...", "==", ">=", "<=", "~=",
	"<number>", "<name>", "<string>", "<eof>",
	NULL
};

#define save_and_next(ls) (save(ls, ls->current), next(ls))
#define MAXSRC          80

static void save (LexState *ls, int c) {
	Mbuffer *b = ls->buff;
	if (b->n + 1 > b->buffsize) {
		size_t newsize;
		if (b->buffsize >= MAX_SIZET/2)
			luaX_lexerror(ls, "lexical element too long", 0);
		newsize = b->buffsize * 2;
		luaZ_resizebuffer(ls->L, b, newsize);
	}
	b->buffer[b->n++] = cast(char, c);
}


void luaX_init (lua_State *L) {
	int i;
	for (i=0; i<NUM_RESERVED; i++) {
		TString *ts = luaS_new(L, luaX_tokens[i]);
		luaS_fix(ts);  /* reserved words are never collected */
		lua_assert(strlen(luaX_tokens[i])+1 <= TOKEN_LEN);
		ts->tsv.reserved = cast_byte(i+1);  /* reserved word */
	}
}

const char *luaX_token2str (LexState *ls, int token) {
	if (token < FIRST_RESERVED) {
		lua_assert(token == cast(unsigned char, token));
		return (iscntrl(token)) ? luaO_pushfstring(ls->L, "char(%d)", token) :
			luaO_pushfstring(ls->L, "%c", token);
	}
	else
		return luaX_tokens[token-FIRST_RESERVED];
}


static const char *txtToken (LexState *ls, int token) {
	switch (token) {
	case TK_NAME:
	case TK_STRING:
	case TK_NUMBER:
		save(ls, '\0');
		return luaZ_buffer(ls->buff);
	default:
		return luaX_token2str(ls, token);
	}
}


void luaX_lexerror (LexState *ls, const char *msg, int token) {
	char buff[MAXSRC];
	luaO_chunkid(buff, getstr(ls->source), MAXSRC);
	msg = luaO_pushfstring(ls->L, "%s:%d: %s", buff, ls->linenumber, msg);
	if (token)
		luaO_pushfstring(ls->L, "%s near " LUA_QS, msg, txtToken(ls, token));
	luaD_throw(ls->L, LUA_ERRSYNTAX);
}


void luaX_syntaxerror (LexState *ls, const char *msg) {
	luaX_lexerror(ls, msg, ls->t.token);
}


TString *luaX_newstring (LexState *ls, const char *str, size_t l) {
	lua_State *L = ls->L;
	TString *ts = luaS_newlstr(L, str, l);
	TValue *o = luaH_setstr(L, ls->fs->h, ts);  /* entry for `str' */
	if (ttisnil(o))
		setbvalue(o, 1);  /* make sure `str' will not be collected */
	return ts;
}


static void inclinenumber (LexState *ls) {
	int old = ls->current;
	lua_assert(currIsNewline(ls));
	next(ls);  /* skip `\n' or `\r' */
	if (currIsNewline(ls) && ls->current != old)
		next(ls);  /* skip `\n\r' or `\r\n' */
	if (++ls->linenumber >= MAX_INT)
		luaX_syntaxerror(ls, "chunk has too many lines");
}


void luaX_setinput (lua_State *L, LexState *ls, ZIO *z, TString *source) {
	ls->decpoint = '.';
	ls->L = L;
	ls->lookahead.token = TK_EOS;  /* no look-ahead token */
	ls->z = z;
	ls->fs = NULL;
	ls->linenumber = 1;
	ls->lastline = 1;
	ls->source = source;
	luaZ_resizebuffer(ls->L, ls->buff, LUA_MINBUFFER);  /* initialize buffer */
	next(ls);  /* read first char */
}



/*
** =======================================================
** LEXICAL ANALYZER
** =======================================================
*/



static int check_next (LexState *ls, const char *set) {
	if (!strchr(set, ls->current))
		return 0;
	save_and_next(ls);
	return 1;
}


static void buffreplace (LexState *ls, char from, char to) {
	size_t n = luaZ_bufflen(ls->buff);
	char *p = luaZ_buffer(ls->buff);
	while (n--)
		if (p[n] == from) p[n] = to;
}


static void trydecpoint (LexState *ls, SemInfo *seminfo) {
	/* format error: try to update decimal point separator */
	struct lconv *cv = localeconv();
	char old = ls->decpoint;
	ls->decpoint = (cv ? cv->decimal_point[0] : '.');
	buffreplace(ls, old, ls->decpoint);  /* try updated decimal separator */
	if (!luaO_str2d(luaZ_buffer(ls->buff), &seminfo->r)) {
		/* format error with correct decimal point: no more options */
		buffreplace(ls, ls->decpoint, '.');  /* undo change (for error message) */
		luaX_lexerror(ls, "malformed number", TK_NUMBER);
	}
}


/* LUA_NUMBER */
static void read_numeral (LexState *ls, SemInfo *seminfo) {
	lua_assert(isdigit(ls->current));
	do {
		save_and_next(ls);
	} while (isdigit(ls->current) || ls->current == '.');
	if (check_next(ls, "Ee"))  /* `E'? */
		check_next(ls, "+-");  /* optional exponent sign */
	while (isalnum(ls->current) || ls->current == '_')
		save_and_next(ls);
	save(ls, '\0');
	buffreplace(ls, '.', ls->decpoint);  /* follow locale for decimal point */
	if (!luaO_str2d(luaZ_buffer(ls->buff), &seminfo->r))  /* format error? */
		trydecpoint(ls, seminfo); /* try to update decimal point separator */
}


static int skip_sep (LexState *ls) {
	int count = 0;
	int s = ls->current;
	lua_assert(s == '[' || s == ']');
	save_and_next(ls);
	while (ls->current == '=') {
		save_and_next(ls);
		count++;
	}
	return (ls->current == s) ? count : (-count) - 1;
}


static void read_long_string (LexState *ls, SemInfo *seminfo, int sep) {
	int cont = 0;
	(void)(cont);  /* avoid warnings when `cont' is not used */
	save_and_next(ls);  /* skip 2nd `[' */
	if (currIsNewline(ls))  /* string starts with a newline? */
		inclinenumber(ls);  /* skip it */
	for (;;) {
		switch (ls->current) {
	  case EOZ:
		  luaX_lexerror(ls, (seminfo) ? "unfinished long string" :
			  "unfinished long comment", TK_EOS);
		  break;  /* to avoid warnings */
#if defined(LUA_COMPAT_LSTR)
	  case '[': {
		  if (skip_sep(ls) == sep) {
			  save_and_next(ls);  /* skip 2nd `[' */
			  cont++;
#if LUA_COMPAT_LSTR == 1
			  if (sep == 0)
				  luaX_lexerror(ls, "nesting of [[...]] is deprecated", '[');
#endif
		  }
		  break;
				}
#endif
	  case ']': {
		  if (skip_sep(ls) == sep) {
			  save_and_next(ls);  /* skip 2nd `]' */
#if defined(LUA_COMPAT_LSTR) && LUA_COMPAT_LSTR == 2
			  cont--;
			  if (sep == 0 && cont >= 0) break;
#endif
			  goto endloop;
		  }
		  break;
				}
	  case '\n':
	  case '\r': {
		  save(ls, '\n');
		  inclinenumber(ls);
		  if (!seminfo) luaZ_resetbuffer(ls->buff);  /* avoid wasting space */
		  break;
				 }
	  default: {
		  if (seminfo) save_and_next(ls);
		  else next(ls);
			   }
		}
	} endloop:
	if (seminfo)
		seminfo->ts = luaX_newstring(ls, luaZ_buffer(ls->buff) + (2 + sep),
		luaZ_bufflen(ls->buff) - 2*(2 + sep));
}


static void read_string (LexState *ls, int del, SemInfo *seminfo) {
	save_and_next(ls);
	while (ls->current != del) {
		switch (ls->current) {
	  case EOZ:
		  luaX_lexerror(ls, "unfinished string", TK_EOS);
		  continue;  /* to avoid warnings */
	  case '\n':
	  case '\r':
		  luaX_lexerror(ls, "unfinished string", TK_STRING);
		  continue;  /* to avoid warnings */
	  case '\\': {
		  int c;
		  next(ls);  /* do not save the `\' */
		  switch (ls->current) {
	  case 'a': c = '\a'; break;
	  case 'b': c = '\b'; break;
	  case 'f': c = '\f'; break;
	  case 'n': c = '\n'; break;
	  case 'r': c = '\r'; break;
	  case 't': c = '\t'; break;
	  case 'v': c = '\v'; break;
	  case '\n':  /* go through */
	  case '\r': save(ls, '\n'); inclinenumber(ls); continue;
	  case EOZ: continue;  /* will raise an error next loop */
	  default: {
		  if (!isdigit(ls->current))
			  save_and_next(ls);  /* handles \\, \", \', and \? */
		  else {  /* \xxx */
			  int i = 0;
			  c = 0;
			  do {
				  c = 10*c + (ls->current-'0');
				  next(ls);
			  } while (++i<3 && isdigit(ls->current));
			  if (c > UCHAR_MAX)
				  luaX_lexerror(ls, "escape sequence too large", TK_STRING);
			  save(ls, c);
		  }
		  continue;
			   }
		  }
		  save(ls, c);
		  next(ls);
		  continue;
				 }
	  default:
		  save_and_next(ls);
		}
	}
	save_and_next(ls);  /* skip delimiter */
	seminfo->ts = luaX_newstring(ls, luaZ_buffer(ls->buff) + 1,
		luaZ_bufflen(ls->buff) - 2);
}


static int llex (LexState *ls, SemInfo *seminfo) {
	luaZ_resetbuffer(ls->buff);
	for (;;) {
		switch (ls->current) {
	  case '\n':
	  case '\r': {
		  inclinenumber(ls);
		  continue;
				 }
	  case '-': {
		  next(ls);
		  if (ls->current != '-') return '-';
		  /* else is a comment */
		  next(ls);
		  if (ls->current == '[') {
			  int sep = skip_sep(ls);
			  luaZ_resetbuffer(ls->buff);  /* `skip_sep' may dirty the buffer */
			  if (sep >= 0) {
				  read_long_string(ls, NULL, sep);  /* long comment */
				  luaZ_resetbuffer(ls->buff);
				  continue;
			  }
		  }
		  /* else short comment */
		  while (!currIsNewline(ls) && ls->current != EOZ)
			  next(ls);
		  continue;
				}
	  case '[': {
		  int sep = skip_sep(ls);
		  if (sep >= 0) {
			  read_long_string(ls, seminfo, sep);
			  return TK_STRING;
		  }
		  else if (sep == -1) return '[';
		  else luaX_lexerror(ls, "invalid long string delimiter", TK_STRING);
				}
	  case '=': {
		  next(ls);
		  if (ls->current != '=') return '=';
		  else { next(ls); return TK_EQ; }
				}
	  case '<': {
		  next(ls);
		  if (ls->current != '=') return '<';
		  else { next(ls); return TK_LE; }
				}
	  case '>': {
		  next(ls);
		  if (ls->current != '=') return '>';
		  else { next(ls); return TK_GE; }
				}
	  case '~': {
		  next(ls);
		  if (ls->current != '=') return '~';
		  else { next(ls); return TK_NE; }
				}
	  case '"':
	  case '\'': {
		  read_string(ls, ls->current, seminfo);
		  return TK_STRING;
				 }
	  case '.': {
		  save_and_next(ls);
		  if (check_next(ls, ".")) {
			  if (check_next(ls, "."))
				  return TK_DOTS;   /* ... */
			  else return TK_CONCAT;   /* .. */
		  }
		  else if (!isdigit(ls->current)) return '.';
		  else {
			  read_numeral(ls, seminfo);
			  return TK_NUMBER;
		  }
				}
	  case EOZ: {
		  return TK_EOS;
				}
	  default: {
		  if (isspace(ls->current)) {
			  lua_assert(!currIsNewline(ls));
			  next(ls);
			  continue;
		  }
		  else if (isdigit(ls->current)) {
			  read_numeral(ls, seminfo);
			  return TK_NUMBER;
		  }
		  else if (isalpha(ls->current) || ls->current == '_') {
			  /* identifier or reserved word */
			  TString *ts;
			  do {
				  save_and_next(ls);
			  } while (isalnum(ls->current) || ls->current == '_');
			  ts = luaX_newstring(ls, luaZ_buffer(ls->buff),
				  luaZ_bufflen(ls->buff));
			  if (ts->tsv.reserved > 0)  /* reserved word? */
				  return ts->tsv.reserved - 1 + FIRST_RESERVED;
			  else {
				  seminfo->ts = ts;
				  return TK_NAME;
			  }
		  }
		  else {
			  int c = ls->current;
			  next(ls);
			  return c;  /* single-char tokens (+ - / ...) */
		  }
			   }
		}
	}
}


void luaX_next (LexState *ls) {
	ls->lastline = ls->linenumber;
	if (ls->lookahead.token != TK_EOS) {  /* is there a look-ahead token? */
		ls->t = ls->lookahead;  /* use this one */
		ls->lookahead.token = TK_EOS;  /* and discharge it */
	}
	else
		ls->t.token = llex(ls, &ls->t.seminfo);  /* read next token */
}


void luaX_lookahead (LexState *ls) {
	lua_assert(ls->lookahead.token == TK_EOS);
	ls->lookahead.token = llex(ls, &ls->lookahead.seminfo);
}


//-------------------------------------------------------------lcode.c------------------------------------------------------
LUAI_FUNC int luaK_codeABx (FuncState *fs, OpCode o, int A, unsigned int Bx);
LUAI_FUNC int luaK_codeABC (FuncState *fs, OpCode o, int A, int B, int C);
LUAI_FUNC void luaK_fixline (FuncState *fs, int line);
LUAI_FUNC void luaK_nil (FuncState *fs, int from, int n);
LUAI_FUNC void luaK_reserveregs (FuncState *fs, int n);
LUAI_FUNC void luaK_checkstack (FuncState *fs, int n);
LUAI_FUNC int luaK_stringK (FuncState *fs, TString *s);
LUAI_FUNC int luaK_numberK (FuncState *fs, lua_Number r);
LUAI_FUNC void luaK_dischargevars (FuncState *fs, expdesc *e);
LUAI_FUNC int luaK_exp2anyreg (FuncState *fs, expdesc *e);
LUAI_FUNC void luaK_exp2nextreg (FuncState *fs, expdesc *e);
LUAI_FUNC void luaK_exp2val (FuncState *fs, expdesc *e);
LUAI_FUNC int luaK_exp2RK (FuncState *fs, expdesc *e);
LUAI_FUNC void luaK_self (FuncState *fs, expdesc *e, expdesc *key);
LUAI_FUNC void luaK_indexed (FuncState *fs, expdesc *t, expdesc *k);
LUAI_FUNC void luaK_goiftrue (FuncState *fs, expdesc *e);
LUAI_FUNC void luaK_storevar (FuncState *fs, expdesc *var, expdesc *e);
LUAI_FUNC void luaK_setreturns (FuncState *fs, expdesc *e, int nresults);
LUAI_FUNC void luaK_setoneret (FuncState *fs, expdesc *e);
LUAI_FUNC int luaK_jump (FuncState *fs);
LUAI_FUNC void luaK_ret (FuncState *fs, int first, int nret);
LUAI_FUNC void luaK_patchlist (FuncState *fs, int list, int target);
LUAI_FUNC void luaK_patchtohere (FuncState *fs, int list);
LUAI_FUNC void luaK_concat (FuncState *fs, int *l1, int l2);
LUAI_FUNC int luaK_getlabel (FuncState *fs);
LUAI_FUNC void luaK_prefix (FuncState *fs, UnOpr op, expdesc *v);
LUAI_FUNC void luaK_infix (FuncState *fs, BinOpr op, expdesc *v);
LUAI_FUNC void luaK_posfix (FuncState *fs, BinOpr op, expdesc *v1, expdesc *v2);
LUAI_FUNC void luaK_setlist (FuncState *fs, int base, int nelems, int tostore);


#define hasjumps(e)	((e)->t != (e)->f)

static int isnumeral(expdesc *e) {
	return (e->k == VKNUM && e->t == NO_JUMP && e->f == NO_JUMP);
}


void luaK_nil (FuncState *fs, int from, int n) {
	Instruction *previous;
	if (fs->pc > fs->lasttarget) {  /* no jumps to current position? */
		if (fs->pc == 0) {  /* function start? */
			if (from >= fs->nactvar)
				return;  /* positions are already clean */
		}
		else {
			previous = &fs->f->code[fs->pc-1];
			if (GET_OPCODE(*previous) == OP_LOADNIL) {
				int pfrom = GETARG_A(*previous);
				int pto = GETARG_B(*previous);
				if (pfrom <= from && from <= pto+1) {  /* can connect both? */
					if (from+n-1 > pto)
						SETARG_B(*previous, from+n-1);
					return;
				}
			}
		}
	}
	luaK_codeABC(fs, OP_LOADNIL, from, from+n-1, 0);  /* else no optimization */
}


int luaK_jump (FuncState *fs) {
	int jpc = fs->jpc;  /* save list of jumps to here */
	int j;
	fs->jpc = NO_JUMP;
	j = luaK_codeAsBx(fs, OP_JMP, 0, NO_JUMP);
	luaK_concat(fs, &j, jpc);  /* keep them on hold */
	return j;
}


void luaK_ret (FuncState *fs, int first, int nret) {
	luaK_codeABC(fs, OP_RETURN, first, nret+1, 0);
}


static int condjump (FuncState *fs, OpCode op, int A, int B, int C) {
	luaK_codeABC(fs, op, A, B, C);
	return luaK_jump(fs);
}


static void fixjump (FuncState *fs, int pc, int dest) {
	Instruction *jmp = &fs->f->code[pc];
	int offset = dest-(pc+1);
	lua_assert(dest != NO_JUMP);
	if (abs(offset) > MAXARG_sBx)
		luaX_syntaxerror(fs->ls, "control structure too long");
	SETARG_sBx(*jmp, offset);
}


/*
** returns current `pc' and marks it as a jump target (to avoid wrong
** optimizations with consecutive instructions not in the same basic block).
*/
int luaK_getlabel (FuncState *fs) {
	fs->lasttarget = fs->pc;
	return fs->pc;
}


static int getjump (FuncState *fs, int pc) {
	int offset = GETARG_sBx(fs->f->code[pc]);
	if (offset == NO_JUMP)  /* point to itself represents end of list */
		return NO_JUMP;  /* end of list */
	else
		return (pc+1)+offset;  /* turn offset into absolute position */
}


static Instruction *getjumpcontrol (FuncState *fs, int pc) {
	Instruction *pi = &fs->f->code[pc];
	if (pc >= 1 && testTMode(GET_OPCODE(*(pi-1))))
		return pi-1;
	else
		return pi;
}


/*
** check whether list has any jump that do not produce a value
** (or produce an inverted value)
*/
static int need_value (FuncState *fs, int list) {
	for (; list != NO_JUMP; list = getjump(fs, list)) {
		Instruction i = *getjumpcontrol(fs, list);
		if (GET_OPCODE(i) != OP_TESTSET) return 1;
	}
	return 0;  /* not found */
}


static int patchtestreg (FuncState *fs, int node, int reg) {
	Instruction *i = getjumpcontrol(fs, node);
	if (GET_OPCODE(*i) != OP_TESTSET)
		return 0;  /* cannot patch other instructions */
	if (reg != NO_REG && reg != GETARG_B(*i))
		SETARG_A(*i, reg);
	else  /* no register to put value or register already has the value */
		*i = CREATE_ABC(OP_TEST, GETARG_B(*i), 0, GETARG_C(*i));

	return 1;
}


static void removevalues (FuncState *fs, int list) {
	for (; list != NO_JUMP; list = getjump(fs, list))
		patchtestreg(fs, list, NO_REG);
}


static void patchlistaux (FuncState *fs, int list, int vtarget, int reg, int dtarget) {
							  while (list != NO_JUMP) {
								  int next = getjump(fs, list);
								  if (patchtestreg(fs, list, reg))
									  fixjump(fs, list, vtarget);
								  else
									  fixjump(fs, list, dtarget);  /* jump to default target */
								  list = next;
							  }
}


static void dischargejpc (FuncState *fs) {
	patchlistaux(fs, fs->jpc, fs->pc, NO_REG, fs->pc);
	fs->jpc = NO_JUMP;
}


void luaK_patchlist (FuncState *fs, int list, int target) {
	if (target == fs->pc)
		luaK_patchtohere(fs, list);
	else {
		lua_assert(target < fs->pc);
		patchlistaux(fs, list, target, NO_REG, target);
	}
}


void luaK_patchtohere (FuncState *fs, int list) {
	luaK_getlabel(fs);
	luaK_concat(fs, &fs->jpc, list);
}


void luaK_concat (FuncState *fs, int *l1, int l2) {
	if (l2 == NO_JUMP) return;
	else if (*l1 == NO_JUMP)
		*l1 = l2;
	else {
		int list = *l1;
		int next;
		while ((next = getjump(fs, list)) != NO_JUMP)  /* find last element */
			list = next;
		fixjump(fs, list, l2);
	}
}


void luaK_checkstack (FuncState *fs, int n) {
	int newstack = fs->freereg + n;
	if (newstack > fs->f->maxstacksize) {
		if (newstack >= MAXSTACK)
			luaX_syntaxerror(fs->ls, "function or expression too complex");
		fs->f->maxstacksize = cast_byte(newstack);
	}
}


void luaK_reserveregs (FuncState *fs, int n) {
	luaK_checkstack(fs, n);
	fs->freereg += n;
}


static void freereg (FuncState *fs, int reg) {
	if (!ISK(reg) && reg >= fs->nactvar) {
		fs->freereg--;
		lua_assert(reg == fs->freereg);
	}
}


static void freeexp (FuncState *fs, expdesc *e) {
	if (e->k == VNONRELOC)
		freereg(fs, e->u.s.info);
}


static int addk (FuncState *fs, TValue *k, TValue *v) {
	lua_State *L = fs->L;
	TValue *idx = luaH_set(L, fs->h, k);
	Proto *f = fs->f;
	int oldsize = f->sizek;
	if (ttisnumber(idx)) {
		lua_assert(luaO_rawequalObj(&fs->f->k[cast_int(nvalue(idx))], v));
		return cast_int(nvalue(idx));
	}
	else {  /* constant not found; create a new entry */
		setnvalue(idx, cast_num(fs->nk));
		luaM_growvector(L, f->k, fs->nk, f->sizek, TValue,
			MAXARG_Bx, "constant table overflow");
		while (oldsize < f->sizek) setnilvalue(&f->k[oldsize++]);
		setobj(L, &f->k[fs->nk], v);
		luaC_barrier(L, f, v);
		return fs->nk++;
	}
}


int luaK_stringK (FuncState *fs, TString *s) {
	TValue o;
	setsvalue(fs->L, &o, s);
	return addk(fs, &o, &o);
}


int luaK_numberK (FuncState *fs, lua_Number r) {
	TValue o;
	setnvalue(&o, r);
	return addk(fs, &o, &o);
}


static int boolK (FuncState *fs, int b) {
	TValue o;
	setbvalue(&o, b);
	return addk(fs, &o, &o);
}


static int nilK (FuncState *fs) {
	TValue k, v;
	setnilvalue(&v);
	/* cannot use nil as key; instead use table itself to represent nil */
	sethvalue(fs->L, &k, fs->h);
	return addk(fs, &k, &v);
}


void luaK_setreturns (FuncState *fs, expdesc *e, int nresults) {
	if (e->k == VCALL) {  /* expression is an open function call? */
		SETARG_C(getcode(fs, e), nresults+1);
	}
	else if (e->k == VVARARG) {
		SETARG_B(getcode(fs, e), nresults+1);
		SETARG_A(getcode(fs, e), fs->freereg);
		luaK_reserveregs(fs, 1);
	}
}


void luaK_setoneret (FuncState *fs, expdesc *e) {
	if (e->k == VCALL) {  /* expression is an open function call? */
		e->k = VNONRELOC;
		e->u.s.info = GETARG_A(getcode(fs, e));
	}
	else if (e->k == VVARARG) {
		SETARG_B(getcode(fs, e), 2);
		e->k = VRELOCABLE;  /* can relocate its simple result */
	}
}


void luaK_dischargevars (FuncState *fs, expdesc *e) {
	switch (e->k) {
	case VLOCAL: {
		e->k = VNONRELOC;
		break;
				 }
	case VUPVAL: {
		e->u.s.info = luaK_codeABC(fs, OP_GETUPVAL, 0, e->u.s.info, 0);
		e->k = VRELOCABLE;
		break;
				 }
	case VGLOBAL: {
		e->u.s.info = luaK_codeABx(fs, OP_GETGLOBAL, 0, e->u.s.info);
		e->k = VRELOCABLE;
		break;
				  }
	case VINDEXED: {
		freereg(fs, e->u.s.aux);
		freereg(fs, e->u.s.info);
		e->u.s.info = luaK_codeABC(fs, OP_GETTABLE, 0, e->u.s.info, e->u.s.aux);
		e->k = VRELOCABLE;
		break;
				   }
	case VVARARG:
	case VCALL: {
		luaK_setoneret(fs, e);
		break;
				}
	default: break;  /* there is one value available (somewhere) */
	}
}


static int code_label (FuncState *fs, int A, int b, int jump) {
	luaK_getlabel(fs);  /* those instructions may be jump targets */
	return luaK_codeABC(fs, OP_LOADBOOL, A, b, jump);
}


static void discharge2reg (FuncState *fs, expdesc *e, int reg) {
	luaK_dischargevars(fs, e);
	switch (e->k) {
	case VNIL: {
		luaK_nil(fs, reg, 1);
		break;
			   }
	case VFALSE:  case VTRUE: {
		luaK_codeABC(fs, OP_LOADBOOL, reg, e->k == VTRUE, 0);
		break;
				  }
	case VK: {
		luaK_codeABx(fs, OP_LOADK, reg, e->u.s.info);
		break;
			 }
	case VKNUM: {
		luaK_codeABx(fs, OP_LOADK, reg, luaK_numberK(fs, e->u.nval));
		break;
				}
	case VRELOCABLE: {
		Instruction *pc = &getcode(fs, e);
		SETARG_A(*pc, reg);
		break;
					 }
	case VNONRELOC: {
		if (reg != e->u.s.info)
			luaK_codeABC(fs, OP_MOVE, reg, e->u.s.info, 0);
		break;
					}
	default: {
		lua_assert(e->k == VVOID || e->k == VJMP);
		return;  /* nothing to do... */
			 }
	}
	e->u.s.info = reg;
	e->k = VNONRELOC;
}


static void discharge2anyreg (FuncState *fs, expdesc *e) {
	if (e->k != VNONRELOC) {
		luaK_reserveregs(fs, 1);
		discharge2reg(fs, e, fs->freereg-1);
	}
}


static void exp2reg (FuncState *fs, expdesc *e, int reg) {
	discharge2reg(fs, e, reg);
	if (e->k == VJMP)
		luaK_concat(fs, &e->t, e->u.s.info);  /* put this jump in `t' list */
	if (hasjumps(e)) {
		int final;  /* position after whole expression */
		int p_f = NO_JUMP;  /* position of an eventual LOAD false */
		int p_t = NO_JUMP;  /* position of an eventual LOAD true */
		if (need_value(fs, e->t) || need_value(fs, e->f)) {
			int fj = (e->k == VJMP) ? NO_JUMP : luaK_jump(fs);
			p_f = code_label(fs, reg, 0, 1);
			p_t = code_label(fs, reg, 1, 0);
			luaK_patchtohere(fs, fj);
		}
		final = luaK_getlabel(fs);
		patchlistaux(fs, e->f, final, reg, p_f);
		patchlistaux(fs, e->t, final, reg, p_t);
	}
	e->f = e->t = NO_JUMP;
	e->u.s.info = reg;
	e->k = VNONRELOC;
}


void luaK_exp2nextreg (FuncState *fs, expdesc *e) {
	luaK_dischargevars(fs, e);
	freeexp(fs, e);
	luaK_reserveregs(fs, 1);
	exp2reg(fs, e, fs->freereg - 1);
}


int luaK_exp2anyreg (FuncState *fs, expdesc *e) {
	luaK_dischargevars(fs, e);
	if (e->k == VNONRELOC) {
		if (!hasjumps(e)) return e->u.s.info;  /* exp is already in a register */
		if (e->u.s.info >= fs->nactvar) {  /* reg. is not a local? */
			exp2reg(fs, e, e->u.s.info);  /* put value on it */
			return e->u.s.info;
		}
	}
	luaK_exp2nextreg(fs, e);  /* default */
	return e->u.s.info;
}


void luaK_exp2val (FuncState *fs, expdesc *e) {
	if (hasjumps(e))
		luaK_exp2anyreg(fs, e);
	else
		luaK_dischargevars(fs, e);
}


int luaK_exp2RK (FuncState *fs, expdesc *e) {
	luaK_exp2val(fs, e);
	switch (e->k) {
	case VKNUM:
	case VTRUE:
	case VFALSE:
	case VNIL: {
		if (fs->nk <= MAXINDEXRK) {  /* constant fit in RK operand? */
			e->u.s.info = (e->k == VNIL)  ? nilK(fs) :
				(e->k == VKNUM) ? luaK_numberK(fs, e->u.nval) :
				boolK(fs, (e->k == VTRUE));
		e->k = VK;
		return RKASK(e->u.s.info);
		}
		else break;
			   }
	case VK: {
		if (e->u.s.info <= MAXINDEXRK)  /* constant fit in argC? */
			return RKASK(e->u.s.info);
		else break;
			 }
	default: break;
	}
	/* not a constant in the right range: put it in a register */
	return luaK_exp2anyreg(fs, e);
}


void luaK_storevar (FuncState *fs, expdesc *var, expdesc *ex) {
	switch (var->k) {
	case VLOCAL: {
		freeexp(fs, ex);
		exp2reg(fs, ex, var->u.s.info);
		return;
				 }
	case VUPVAL: {
		int e = luaK_exp2anyreg(fs, ex);
		luaK_codeABC(fs, OP_SETUPVAL, e, var->u.s.info, 0);
		break;
				 }
	case VGLOBAL: {
		int e = luaK_exp2anyreg(fs, ex);
		luaK_codeABx(fs, OP_SETGLOBAL, e, var->u.s.info);
		break;
				  }
	case VINDEXED: {
		int e = luaK_exp2RK(fs, ex);
		luaK_codeABC(fs, OP_SETTABLE, var->u.s.info, var->u.s.aux, e);
		break;
				   }
	default: {
		lua_assert(0);  /* invalid var kind to store */
		break;
			 }
	}
	freeexp(fs, ex);
}


void luaK_self (FuncState *fs, expdesc *e, expdesc *key) {
	int func;
	luaK_exp2anyreg(fs, e);
	freeexp(fs, e);
	func = fs->freereg;
	luaK_reserveregs(fs, 2);
	luaK_codeABC(fs, OP_SELF, func, e->u.s.info, luaK_exp2RK(fs, key));
	freeexp(fs, key);
	e->u.s.info = func;
	e->k = VNONRELOC;
}


static void invertjump (FuncState *fs, expdesc *e) {
	Instruction *pc = getjumpcontrol(fs, e->u.s.info);
	lua_assert(testTMode(GET_OPCODE(*pc)) && GET_OPCODE(*pc) != OP_TESTSET &&
		GET_OPCODE(*pc) != OP_TEST);
	SETARG_A(*pc, !(GETARG_A(*pc)));
}


static int jumponcond (FuncState *fs, expdesc *e, int cond) {
	if (e->k == VRELOCABLE) {
		Instruction ie = getcode(fs, e);
		if (GET_OPCODE(ie) == OP_NOT) {
			fs->pc--;  /* remove previous OP_NOT */
			return condjump(fs, OP_TEST, GETARG_B(ie), 0, !cond);
		}
		/* else go through */
	}
	discharge2anyreg(fs, e);
	freeexp(fs, e);
	return condjump(fs, OP_TESTSET, NO_REG, e->u.s.info, cond);
}


void luaK_goiftrue (FuncState *fs, expdesc *e) {
	int pc;  /* pc of last jump */
	luaK_dischargevars(fs, e);
	switch (e->k) {
	case VK: case VKNUM: case VTRUE: {
		pc = NO_JUMP;  /* always true; do nothing */
		break;
			 }
	case VFALSE: {
		pc = luaK_jump(fs);  /* always jump */
		break;
				 }
	case VJMP: {
		invertjump(fs, e);
		pc = e->u.s.info;
		break;
			   }
	default: {
		pc = jumponcond(fs, e, 0);
		break;
			 }
	}
	luaK_concat(fs, &e->f, pc);  /* insert last jump in `f' list */
	luaK_patchtohere(fs, e->t);
	e->t = NO_JUMP;
}


static void luaK_goiffalse (FuncState *fs, expdesc *e) {
	int pc;  /* pc of last jump */
	luaK_dischargevars(fs, e);
	switch (e->k) {
	case VNIL: case VFALSE: {
		pc = NO_JUMP;  /* always false; do nothing */
		break;
			   }
	case VTRUE: {
		pc = luaK_jump(fs);  /* always jump */
		break;
				}
	case VJMP: {
		pc = e->u.s.info;
		break;
			   }
	default: {
		pc = jumponcond(fs, e, 1);
		break;
			 }
	}
	luaK_concat(fs, &e->t, pc);  /* insert last jump in `t' list */
	luaK_patchtohere(fs, e->f);
	e->f = NO_JUMP;
}


static void codenot (FuncState *fs, expdesc *e) {
	luaK_dischargevars(fs, e);
	switch (e->k) {
	case VNIL: case VFALSE: {
		e->k = VTRUE;
		break;
			   }
	case VK: case VKNUM: case VTRUE: {
		e->k = VFALSE;
		break;
			 }
	case VJMP: {
		invertjump(fs, e);
		break;
			   }
	case VRELOCABLE:
	case VNONRELOC: {
		discharge2anyreg(fs, e);
		freeexp(fs, e);
		e->u.s.info = luaK_codeABC(fs, OP_NOT, 0, e->u.s.info, 0);
		e->k = VRELOCABLE;
		break;
					}
	default: {
		lua_assert(0);  /* cannot happen */
		break;
			 }
	}
	/* interchange true and false lists */
	{ int temp = e->f; e->f = e->t; e->t = temp; }
	removevalues(fs, e->f);
	removevalues(fs, e->t);
}


void luaK_indexed (FuncState *fs, expdesc *t, expdesc *k) {
	t->u.s.aux = luaK_exp2RK(fs, k);
	t->k = VINDEXED;
}


static int constfolding (OpCode op, expdesc *e1, expdesc *e2) {
	lua_Number v1, v2, r;
	if (!isnumeral(e1) || !isnumeral(e2)) return 0;
	v1 = e1->u.nval;
	v2 = e2->u.nval;
	switch (op) {
	case OP_ADD: r = luai_numadd(v1, v2); break;
	case OP_SUB: r = luai_numsub(v1, v2); break;
	case OP_MUL: r = luai_nummul(v1, v2); break;
	case OP_DIV:
		if (v2 == 0) return 0;  /* do not attempt to divide by 0 */
		r = luai_numdiv(v1, v2); break;
	case OP_MOD:
		if (v2 == 0) return 0;  /* do not attempt to divide by 0 */
		r = luai_nummod(v1, v2); break;
	case OP_POW: r = luai_numpow(v1, v2); break;
	case OP_UNM: r = luai_numunm(v1); break;
	case OP_LEN: return 0;  /* no constant folding for 'len' */
	default: lua_assert(0); r = 0; break;
	}
	if (luai_numisnan(r)) return 0;  /* do not attempt to produce NaN */
	e1->u.nval = r;
	return 1;
}


static void codearith (FuncState *fs, OpCode op, expdesc *e1, expdesc *e2) {
	if (constfolding(op, e1, e2))
		return;
	else {
		int o2 = (op != OP_UNM && op != OP_LEN) ? luaK_exp2RK(fs, e2) : 0;
		int o1 = luaK_exp2RK(fs, e1);
		if (o1 > o2) {
			freeexp(fs, e1);
			freeexp(fs, e2);
		}
		else {
			freeexp(fs, e2);
			freeexp(fs, e1);
		}
		e1->u.s.info = luaK_codeABC(fs, op, 0, o1, o2);
		e1->k = VRELOCABLE;
	}
}


static void codecomp (FuncState *fs, OpCode op, int cond, expdesc *e1, expdesc *e2) {
						  int o1 = luaK_exp2RK(fs, e1);
						  int o2 = luaK_exp2RK(fs, e2);
						  freeexp(fs, e2);
						  freeexp(fs, e1);
						  if (cond == 0 && op != OP_EQ) {
							  int temp;  /* exchange args to replace by `<' or `<=' */
							  temp = o1; o1 = o2; o2 = temp;  /* o1 <==> o2 */
							  cond = 1;
						  }
						  e1->u.s.info = condjump(fs, op, cond, o1, o2);
						  e1->k = VJMP;
}


void luaK_prefix (FuncState *fs, UnOpr op, expdesc *e) {
	expdesc e2;
	e2.t = e2.f = NO_JUMP; e2.k = VKNUM; e2.u.nval = 0;
	switch (op) {
	case OPR_MINUS: {
		if (e->k == VK)
			luaK_exp2anyreg(fs, e);  /* cannot operate on non-numeric constants */
		codearith(fs, OP_UNM, e, &e2);
		break;
					}
	case OPR_NOT: codenot(fs, e); break;
	case OPR_LEN: {
		luaK_exp2anyreg(fs, e);  /* cannot operate on constants */
		codearith(fs, OP_LEN, e, &e2);
		break;
				  }
	default: lua_assert(0);
	}
}


void luaK_infix (FuncState *fs, BinOpr op, expdesc *v) {
	switch (op) {
	case OPR_AND: {
		luaK_goiftrue(fs, v);
		break;
				  }
	case OPR_OR: {
		luaK_goiffalse(fs, v);
		break;
				 }
	case OPR_CONCAT: {
		luaK_exp2nextreg(fs, v);  /* operand must be on the `stack' */
		break;
					 }
	case OPR_ADD: case OPR_SUB: case OPR_MUL: case OPR_DIV:
	case OPR_MOD: case OPR_POW: {
		if (!isnumeral(v)) luaK_exp2RK(fs, v);
		break;
				  }
	default: {
		luaK_exp2RK(fs, v);
		break;
			 }
	}
}


void luaK_posfix (FuncState *fs, BinOpr op, expdesc *e1, expdesc *e2) {
	switch (op) {
	case OPR_AND: {
		lua_assert(e1->t == NO_JUMP);  /* list must be closed */
		luaK_dischargevars(fs, e2);
		luaK_concat(fs, &e2->f, e1->f);
		*e1 = *e2;
		break;
				  }
	case OPR_OR: {
		lua_assert(e1->f == NO_JUMP);  /* list must be closed */
		luaK_dischargevars(fs, e2);
		luaK_concat(fs, &e2->t, e1->t);
		*e1 = *e2;
		break;
				 }
	case OPR_CONCAT: {
		luaK_exp2val(fs, e2);
		if (e2->k == VRELOCABLE && GET_OPCODE(getcode(fs, e2)) == OP_CONCAT) {
			lua_assert(e1->u.s.info == GETARG_B(getcode(fs, e2))-1);
			freeexp(fs, e1);
			SETARG_B(getcode(fs, e2), e1->u.s.info);
			e1->k = VRELOCABLE; e1->u.s.info = e2->u.s.info;
		}
		else {
			luaK_exp2nextreg(fs, e2);  /* operand must be on the 'stack' */
			codearith(fs, OP_CONCAT, e1, e2);
		}
		break;
					 }
	case OPR_ADD: codearith(fs, OP_ADD, e1, e2); break;
	case OPR_SUB: codearith(fs, OP_SUB, e1, e2); break;
	case OPR_MUL: codearith(fs, OP_MUL, e1, e2); break;
	case OPR_DIV: codearith(fs, OP_DIV, e1, e2); break;
	case OPR_MOD: codearith(fs, OP_MOD, e1, e2); break;
	case OPR_POW: codearith(fs, OP_POW, e1, e2); break;
	case OPR_EQ: codecomp(fs, OP_EQ, 1, e1, e2); break;
	case OPR_NE: codecomp(fs, OP_EQ, 0, e1, e2); break;
	case OPR_LT: codecomp(fs, OP_LT, 1, e1, e2); break;
	case OPR_LE: codecomp(fs, OP_LE, 1, e1, e2); break;
	case OPR_GT: codecomp(fs, OP_LT, 0, e1, e2); break;
	case OPR_GE: codecomp(fs, OP_LE, 0, e1, e2); break;
	default: lua_assert(0);
	}
}


void luaK_fixline (FuncState *fs, int line) {
	fs->f->lineinfo[fs->pc - 1] = line;
}


static int luaK_code (FuncState *fs, Instruction i, int line) {
	Proto *f = fs->f;
	dischargejpc(fs);  /* `pc' will change */
	/* put new instruction in code array */
	luaM_growvector(fs->L, f->code, fs->pc, f->sizecode, Instruction,
		MAX_INT, "code size overflow");
	f->code[fs->pc] = i;
	/* save corresponding line information */
	luaM_growvector(fs->L, f->lineinfo, fs->pc, f->sizelineinfo, int,
		MAX_INT, "code size overflow");
	f->lineinfo[fs->pc] = line;
	return fs->pc++;
}


int luaK_codeABC (FuncState *fs, OpCode o, int a, int b, int c) {
	lua_assert(getOpMode(o) == iABC);
	lua_assert(getBMode(o) != OpArgN || b == 0);
	lua_assert(getCMode(o) != OpArgN || c == 0);
	return luaK_code(fs, CREATE_ABC(o, a, b, c), fs->ls->lastline);
}


int luaK_codeABx (FuncState *fs, OpCode o, int a, unsigned int bc) {
	lua_assert(getOpMode(o) == iABx || getOpMode(o) == iAsBx);
	lua_assert(getCMode(o) == OpArgN);
	return luaK_code(fs, CREATE_ABx(o, a, bc), fs->ls->lastline);
}


void luaK_setlist (FuncState *fs, int base, int nelems, int tostore) {
	int c =  (nelems - 1)/LFIELDS_PER_FLUSH + 1;
	int b = (tostore == LUA_MULTRET) ? 0 : tostore;
	lua_assert(tostore != 0);
	if (c <= MAXARG_C)
		luaK_codeABC(fs, OP_SETLIST, base, b, c);
	else {
		luaK_codeABC(fs, OP_SETLIST, base, b, 0);
		luaK_code(fs, cast(Instruction, c), fs->ls->lastline);
	}
	fs->freereg = base + 1;  /* free registers with list values */
}

//-------------------------------------------------------------lparser.c----------------------------------------------------
//typedef struct LG {
//	lua_State l;
//	global_State g;
//} LG;

/*
** $Id: lparser.c,v 2.42a 2006/06/05 15:57:59 roberto Exp $
** Lua Parser
** See Copyright Notice in lua.h
*/


//#include <string.h>

#define lparser_c
#define LUA_CORE

//#include "lua.h"

//#include "lcode.h"
//#include "ldebug.h"
//#include "ldo.h"
//#include "lfunc.h"
//#include "llex.h"
//#include "lmem.h"
//#include "lobject.h"
//#include "lopcodes.h"
//#include "lparser.h"
//#include "lstate.h"
//#include "lstring.h"
//#include "ltable.h"



#define hasmultret(k)		((k) == VCALL || (k) == VVARARG)

#define getlocvar(fs, i)	((fs)->f->locvars[(fs)->actvar[i]])

#define luaY_checklimit(fs,v,l,m)	if ((v)>(l)) errorlimit(fs,l,m)

/*
** nodes for block list (list of active blocks)
*/
typedef struct BlockCnt {
	struct BlockCnt *previous;  /* chain */
	int breaklist;  /* list of jumps out of this loop */
	lu_byte nactvar;  /* # active locals outside the breakable structure */
	lu_byte upval;  /* true if some variable in the block is an upvalue */
	lu_byte isbreakable;  /* true if `block' is a loop */
} BlockCnt;



/*
** prototypes for recursive non-terminal functions
*/
static void chunk (LexState *ls);
static void expr (LexState *ls, expdesc *v);

static void anchor_token (LexState *ls) {
	if (ls->t.token == TK_NAME || ls->t.token == TK_STRING) {
		TString *ts = ls->t.seminfo.ts;
		luaX_newstring(ls, getstr(ts), ts->tsv.len);
	}
}


static void error_expected (LexState *ls, int token) {
	luaX_syntaxerror(ls,
		luaO_pushfstring(ls->L, LUA_QS " expected", luaX_token2str(ls, token)));
}


static void errorlimit (FuncState *fs, int limit, const char *what) {
	const char *msg = (fs->f->linedefined == 0) ?
		luaO_pushfstring(fs->L, "main function has more than %d %s", limit, what) :
	luaO_pushfstring(fs->L, "function at line %d has more than %d %s",
		fs->f->linedefined, limit, what);
	luaX_lexerror(fs->ls, msg, 0);
}


static int testnext (LexState *ls, int c) {
	if (ls->t.token == c) {
		luaX_next(ls);
		return 1;
	}
	else return 0;
}


static void check (LexState *ls, int c) {
	if (ls->t.token != c)
		error_expected(ls, c);
}

static void checknext (LexState *ls, int c) {
	check(ls, c);
	luaX_next(ls);
}


#define check_condition(ls,c,msg)	{ if (!(c)) luaX_syntaxerror(ls, msg); }



static void check_match (LexState *ls, int what, int who, int where) {
	if (!testnext(ls, what)) {
		if (where == ls->linenumber)
			error_expected(ls, what);
		else {
			luaX_syntaxerror(ls, luaO_pushfstring(ls->L,
				LUA_QS " expected (to close " LUA_QS " at line %d)",
				luaX_token2str(ls, what), luaX_token2str(ls, who), where));
		}
	}
}


static TString *str_checkname (LexState *ls) {
	TString *ts;
	check(ls, TK_NAME);
	ts = ls->t.seminfo.ts;
	luaX_next(ls);
	return ts;
}


static void init_exp (expdesc *e, expkind k, int i) {
	e->f = e->t = NO_JUMP;
	e->k = k;
	e->u.s.info = i;
}


static void codestring (LexState *ls, expdesc *e, TString *s) {
	init_exp(e, VK, luaK_stringK(ls->fs, s));
}


static void checkname(LexState *ls, expdesc *e) {
	codestring(ls, e, str_checkname(ls));
}


static int registerlocalvar (LexState *ls, TString *varname) {
	FuncState *fs = ls->fs;
	Proto *f = fs->f;
	int oldsize = f->sizelocvars;
	luaM_growvector(ls->L, f->locvars, fs->nlocvars, f->sizelocvars,
		LocVar, SHRT_MAX, "too many local variables");
	while (oldsize < f->sizelocvars) f->locvars[oldsize++].varname = NULL;
	f->locvars[fs->nlocvars].varname = varname;
	luaC_objbarrier(ls->L, f, varname);
	return fs->nlocvars++;
}


#define new_localvarliteral(ls,v,n) \
	new_localvar(ls, luaX_newstring(ls, "" v, (sizeof(v)/sizeof(char))-1), n)


static void new_localvar (LexState *ls, TString *name, int n) {
	FuncState *fs = ls->fs;
	luaY_checklimit(fs, fs->nactvar+n+1, LUAI_MAXVARS, "local variables");
	fs->actvar[fs->nactvar+n] = cast(unsigned short, registerlocalvar(ls, name));
}


static void adjustlocalvars (LexState *ls, int nvars) {
	FuncState *fs = ls->fs;
	fs->nactvar = cast_byte(fs->nactvar + nvars);
	for (; nvars; nvars--) {
		getlocvar(fs, fs->nactvar - nvars).startpc = fs->pc;
	}
}


static void removevars (LexState *ls, int tolevel) {
	FuncState *fs = ls->fs;
	while (fs->nactvar > tolevel)
		getlocvar(fs, --fs->nactvar).endpc = fs->pc;
}


static int indexupvalue (FuncState *fs, TString *name, expdesc *v) {
	int i;
	Proto *f = fs->f;
	int oldsize = f->sizeupvalues;
	for (i=0; i<f->nups; i++) {
		if (fs->upvalues[i].k == v->k && fs->upvalues[i].info == v->u.s.info) {
			lua_assert(f->upvalues[i] == name);
			return i;
		}
	}
	/* new one */
	luaY_checklimit(fs, f->nups + 1, LUAI_MAXUPVALUES, "upvalues");
	luaM_growvector(fs->L, f->upvalues, f->nups, f->sizeupvalues,
		TString *, MAX_INT, "");
	while (oldsize < f->sizeupvalues) f->upvalues[oldsize++] = NULL;
	f->upvalues[f->nups] = name;
	luaC_objbarrier(fs->L, f, name);
	lua_assert(v->k == VLOCAL || v->k == VUPVAL);
	fs->upvalues[f->nups].k = cast_byte(v->k);
	fs->upvalues[f->nups].info = cast_byte(v->u.s.info);
	return f->nups++;
}


static int searchvar (FuncState *fs, TString *n) {
	int i;
	for (i=fs->nactvar-1; i >= 0; i--) {
		if (n == getlocvar(fs, i).varname)
			return i;
	}
	return -1;  /* not found */
}


static void markupval (FuncState *fs, int level) {
	BlockCnt *bl = fs->bl;
	while (bl && bl->nactvar > level) bl = bl->previous;
	if (bl) bl->upval = 1;
}


static int singlevaraux (FuncState *fs, TString *n, expdesc *var, int base) {
	if (fs == NULL) {  /* no more levels? */
		init_exp(var, VGLOBAL, NO_REG);  /* default is global variable */
		return VGLOBAL;
	}
	else {
		int v = searchvar(fs, n);  /* look up at current level */
		if (v >= 0) {
			init_exp(var, VLOCAL, v);
			if (!base)
				markupval(fs, v);  /* local will be used as an upval */
			return VLOCAL;
		}
		else {  /* not found at current level; try upper one */
			if (singlevaraux(fs->prev, n, var, 0) == VGLOBAL)
				return VGLOBAL;
			var->u.s.info = indexupvalue(fs, n, var);  /* else was LOCAL or UPVAL */
			var->k = VUPVAL;  /* upvalue in this level */
			return VUPVAL;
		}
	}
}


static void singlevar (LexState *ls, expdesc *var) {
	TString *varname = str_checkname(ls);
	FuncState *fs = ls->fs;
	if (singlevaraux(fs, varname, var, 1) == VGLOBAL)
		var->u.s.info = luaK_stringK(fs, varname);  /* info points to global name */
}


static void adjust_assign (LexState *ls, int nvars, int nexps, expdesc *e) {
	FuncState *fs = ls->fs;
	int extra = nvars - nexps;
	if (hasmultret(e->k)) {
		extra++;  /* includes call itself */
		if (extra < 0) extra = 0;
		luaK_setreturns(fs, e, extra);  /* last exp. provides the difference */
		if (extra > 1) luaK_reserveregs(fs, extra-1);
	}
	else {
		if (e->k != VVOID) luaK_exp2nextreg(fs, e);  /* close last expression */
		if (extra > 0) {
			int reg = fs->freereg;
			luaK_reserveregs(fs, extra);
			luaK_nil(fs, reg, extra);
		}
	}
}


static void enterlevel (LexState *ls) {
	if (++ls->L->nCcalls > LUAI_MAXCCALLS)
		luaX_lexerror(ls, "chunk has too many syntax levels", 0);
}


#define leavelevel(ls)	((ls)->L->nCcalls--)


static void enterblock (FuncState *fs, BlockCnt *bl, lu_byte isbreakable) {
	bl->breaklist = NO_JUMP;
	bl->isbreakable = isbreakable;
	bl->nactvar = fs->nactvar;
	bl->upval = 0;
	bl->previous = fs->bl;
	fs->bl = bl;
	lua_assert(fs->freereg == fs->nactvar);
}


static void leaveblock (FuncState *fs) {
	BlockCnt *bl = fs->bl;
	fs->bl = bl->previous;
	removevars(fs->ls, bl->nactvar);
	if (bl->upval)
		luaK_codeABC(fs, OP_CLOSE, bl->nactvar, 0, 0);
	/* a block either controls scope or breaks (never both) */
	lua_assert(!bl->isbreakable || !bl->upval);
	lua_assert(bl->nactvar == fs->nactvar);
	fs->freereg = fs->nactvar;  /* free registers */
	luaK_patchtohere(fs, bl->breaklist);
}


static void pushclosure (LexState *ls, FuncState *func, expdesc *v) {
	FuncState *fs = ls->fs;
	Proto *f = fs->f;
	int oldsize = f->sizep;
	int i;
	luaM_growvector(ls->L, f->p, fs->np, f->sizep, Proto *,
		MAXARG_Bx, "constant table overflow");
	while (oldsize < f->sizep) f->p[oldsize++] = NULL;
	f->p[fs->np++] = func->f;
	luaC_objbarrier(ls->L, f, func->f);
	init_exp(v, VRELOCABLE, luaK_codeABx(fs, OP_CLOSURE, 0, fs->np-1));
	for (i=0; i<func->f->nups; i++) {
		OpCode o = (func->upvalues[i].k == VLOCAL) ? OP_MOVE : OP_GETUPVAL;
		luaK_codeABC(fs, o, 0, func->upvalues[i].info, 0);
	}
}


static void open_func (LexState *ls, FuncState *fs) {
	lua_State *L = ls->L;
	Proto *f = luaF_newproto(L);
	fs->f = f;
	fs->prev = ls->fs;  /* linked list of funcstates */
	fs->ls = ls;
	fs->L = L;
	ls->fs = fs;
	fs->pc = 0;
	fs->lasttarget = -1;
	fs->jpc = NO_JUMP;
	fs->freereg = 0;
	fs->nk = 0;
	fs->np = 0;
	fs->nlocvars = 0;
	fs->nactvar = 0;
	fs->bl = NULL;
	f->source = ls->source;
	f->maxstacksize = 2;  /* registers 0/1 are always valid */
	fs->h = luaH_new(L, 0, 0);
	/* anchor table of constants and prototype (to avoid being collected) */
	sethvalue2s(L, L->top, fs->h);
	incr_top(L);
	setptvalue2s(L, L->top, f);
	incr_top(L);
}


static void close_func (LexState *ls) {
	lua_State *L = ls->L;
	FuncState *fs = ls->fs;
	Proto *f = fs->f;
	removevars(ls, 0);
	luaK_ret(fs, 0, 0);  /* final return */
	luaM_reallocvector(L, f->code, f->sizecode, fs->pc, Instruction);
	f->sizecode = fs->pc;
	luaM_reallocvector(L, f->lineinfo, f->sizelineinfo, fs->pc, int);
	f->sizelineinfo = fs->pc;
	luaM_reallocvector(L, f->k, f->sizek, fs->nk, TValue);
	f->sizek = fs->nk;
	luaM_reallocvector(L, f->p, f->sizep, fs->np, Proto *);
	f->sizep = fs->np;
	luaM_reallocvector(L, f->locvars, f->sizelocvars, fs->nlocvars, LocVar);
	f->sizelocvars = fs->nlocvars;
	luaM_reallocvector(L, f->upvalues, f->sizeupvalues, f->nups, TString *);
	f->sizeupvalues = f->nups;
	lua_assert(luaG_checkcode(f));
	lua_assert(fs->bl == NULL);
	ls->fs = fs->prev;
	L->top -= 2;  /* remove table and prototype from the stack */
	/* last token read was anchored in defunct function; must reanchor it */
	if (fs) anchor_token(ls);
}


Proto *luaY_parser (lua_State *L, ZIO *z, Mbuffer *buff, const char *name) {
	struct LexState lexstate;
	struct FuncState funcstate;
	lexstate.buff = buff;
	luaX_setinput(L, &lexstate, z, luaS_new(L, name));
	open_func(&lexstate, &funcstate);
	funcstate.f->is_vararg = VARARG_ISVARARG;  /* main func. is always vararg */
	luaX_next(&lexstate);  /* read first token */
	chunk(&lexstate);
	check(&lexstate, TK_EOS);
	close_func(&lexstate);
	lua_assert(funcstate.prev == NULL);
	lua_assert(funcstate.f->nups == 0);
	lua_assert(lexstate.fs == NULL);
	return funcstate.f;
}



/*============================================================*/
/* GRAMMAR RULES */
/*============================================================*/


static void field (LexState *ls, expdesc *v) {
	/* field -> ['.' | ':'] NAME */
	FuncState *fs = ls->fs;
	expdesc key;
	luaK_exp2anyreg(fs, v);
	luaX_next(ls);  /* skip the dot or colon */
	checkname(ls, &key);
	luaK_indexed(fs, v, &key);
}


static void yindex (LexState *ls, expdesc *v) {
	/* index -> '[' expr ']' */
	luaX_next(ls);  /* skip the '[' */
	expr(ls, v);
	luaK_exp2val(ls->fs, v);
	checknext(ls, ']');
}


/*
** {======================================================================
** Rules for Constructors
** =======================================================================
*/


struct ConsControl {
	expdesc v;  /* last list item read */
	expdesc *t;  /* table descriptor */
	int nh;  /* total number of `record' elements */
	int na;  /* total number of array elements */
	int tostore;  /* number of array elements pending to be stored */
};


static void recfield (LexState *ls, struct ConsControl *cc) {
	/* recfield -> (NAME | `['exp1`]') = exp1 */
	FuncState *fs = ls->fs;
	int reg = ls->fs->freereg;
	expdesc key, val;
	int rkkey;
	if (ls->t.token == TK_NAME) {
		luaY_checklimit(fs, cc->nh, MAX_INT, "items in a constructor");
		checkname(ls, &key);
	}
	else  /* ls->t.token == '[' */
		yindex(ls, &key);
	cc->nh++;
	checknext(ls, '=');
	rkkey = luaK_exp2RK(fs, &key);
	expr(ls, &val);
	luaK_codeABC(fs, OP_SETTABLE, cc->t->u.s.info, rkkey, luaK_exp2RK(fs, &val));
	fs->freereg = reg;  /* free registers */
}


static void closelistfield (FuncState *fs, struct ConsControl *cc) {
	if (cc->v.k == VVOID) return;  /* there is no list item */
	luaK_exp2nextreg(fs, &cc->v);
	cc->v.k = VVOID;
	if (cc->tostore == LFIELDS_PER_FLUSH) {
		luaK_setlist(fs, cc->t->u.s.info, cc->na, cc->tostore);  /* flush */
		cc->tostore = 0;  /* no more items pending */
	}
}


static void lastlistfield (FuncState *fs, struct ConsControl *cc) {
	if (cc->tostore == 0) return;
	if (hasmultret(cc->v.k)) {
		luaK_setmultret(fs, &cc->v);
		luaK_setlist(fs, cc->t->u.s.info, cc->na, LUA_MULTRET);
		cc->na--;  /* do not count last expression (unknown number of elements) */
	}
	else {
		if (cc->v.k != VVOID)
			luaK_exp2nextreg(fs, &cc->v);
		luaK_setlist(fs, cc->t->u.s.info, cc->na, cc->tostore);
	}
}


static void listfield (LexState *ls, struct ConsControl *cc) {
	expr(ls, &cc->v);
	luaY_checklimit(ls->fs, cc->na, MAX_INT, "items in a constructor");
	cc->na++;
	cc->tostore++;
}


static void constructor (LexState *ls, expdesc *t) {
	/* constructor -> ?? */
	FuncState *fs = ls->fs;
	int line = ls->linenumber;
	int pc = luaK_codeABC(fs, OP_NEWTABLE, 0, 0, 0);
	struct ConsControl cc;
	cc.na = cc.nh = cc.tostore = 0;
	cc.t = t;
	init_exp(t, VRELOCABLE, pc);
	init_exp(&cc.v, VVOID, 0);  /* no value (yet) */
	luaK_exp2nextreg(ls->fs, t);  /* fix it at stack top (for gc) */
	checknext(ls, '{');
	do {
		lua_assert(cc.v.k == VVOID || cc.tostore > 0);
		if (ls->t.token == '}') break;
		closelistfield(fs, &cc);
		switch(ls->t.token) {
	  case TK_NAME: {  /* may be listfields or recfields */
		  luaX_lookahead(ls);
		  if (ls->lookahead.token != '=')  /* expression? */
			  listfield(ls, &cc);
		  else
			  recfield(ls, &cc);
		  break;
					}
	  case '[': {  /* constructor_item -> recfield */
		  recfield(ls, &cc);
		  break;
				}
	  default: {  /* constructor_part -> listfield */
		  listfield(ls, &cc);
		  break;
			   }
		}
	} while (testnext(ls, ',') || testnext(ls, ';'));
	check_match(ls, '}', '{', line);
	lastlistfield(fs, &cc);
	SETARG_B(fs->f->code[pc], luaO_int2fb(cc.na)); /* set initial array size */
	SETARG_C(fs->f->code[pc], luaO_int2fb(cc.nh));  /* set initial table size */
}

/* }====================================================================== */



static void parlist (LexState *ls) {
	/* parlist -> [ param { `,' param } ] */
	FuncState *fs = ls->fs;
	Proto *f = fs->f;
	int nparams = 0;
	f->is_vararg = 0;
	if (ls->t.token != ')') {  /* is `parlist' not empty? */
		do {
			switch (ls->t.token) {
		case TK_NAME: {  /* param -> NAME */
			new_localvar(ls, str_checkname(ls), nparams++);
			break;
					  }
		case TK_DOTS: {  /* param -> `...' */
			luaX_next(ls);
#if defined(LUA_COMPAT_VARARG)
			/* use `arg' as default name */
			new_localvarliteral(ls, "arg", nparams++);
			f->is_vararg = VARARG_HASARG | VARARG_NEEDSARG;
#endif
			f->is_vararg |= VARARG_ISVARARG;
			break;
					  }
		default: luaX_syntaxerror(ls, "<name> or " LUA_QL("...") " expected");
			}
		} while (!f->is_vararg && testnext(ls, ','));
	}
	adjustlocalvars(ls, nparams);
	f->numparams = cast_byte(fs->nactvar - (f->is_vararg & VARARG_HASARG));
	luaK_reserveregs(fs, fs->nactvar);  /* reserve register for parameters */
}


static void body (LexState *ls, expdesc *e, int needself, int line) {
	/* body ->  `(' parlist `)' chunk END */
	FuncState new_fs;
	open_func(ls, &new_fs);
	new_fs.f->linedefined = line;
	checknext(ls, '(');
	if (needself) {
		new_localvarliteral(ls, "self", 0);
		adjustlocalvars(ls, 1);
	}
	parlist(ls);
	checknext(ls, ')');
	chunk(ls);
	new_fs.f->lastlinedefined = ls->linenumber;
	check_match(ls, TK_END, TK_FUNCTION, line);
	close_func(ls);
	pushclosure(ls, &new_fs, e);
}


static int explist1 (LexState *ls, expdesc *v) {
	/* explist1 -> expr { `,' expr } */
	int n = 1;  /* at least one expression */
	expr(ls, v);
	while (testnext(ls, ',')) {
		luaK_exp2nextreg(ls->fs, v);
		expr(ls, v);
		n++;
	}
	return n;
}


static void funcargs (LexState *ls, expdesc *f) {
	FuncState *fs = ls->fs;
	expdesc args;
	int base, nparams;
	int line = ls->linenumber;
	switch (ls->t.token) {
	case '(': {  /* funcargs -> `(' [ explist1 ] `)' */
		if (line != ls->lastline)
			luaX_syntaxerror(ls,"ambiguous syntax (function call x new statement)");
		luaX_next(ls);
		if (ls->t.token == ')')  /* arg list is empty? */
			args.k = VVOID;
		else {
			explist1(ls, &args);
			luaK_setmultret(fs, &args);
		}
		check_match(ls, ')', '(', line);
		break;
			  }
	case '{': {  /* funcargs -> constructor */
		constructor(ls, &args);
		break;
			  }
	case TK_STRING: {  /* funcargs -> STRING */
		codestring(ls, &args, ls->t.seminfo.ts);
		luaX_next(ls);  /* must use `seminfo' before `next' */
		break;
					}
	default: {
		luaX_syntaxerror(ls, "function arguments expected");
		return;
			 }
	}
	lua_assert(f->k == VNONRELOC);
	base = f->u.s.info;  /* base register for call */
	if (hasmultret(args.k))
		nparams = LUA_MULTRET;  /* open call */
	else {
		if (args.k != VVOID)
			luaK_exp2nextreg(fs, &args);  /* close last argument */
		nparams = fs->freereg - (base+1);
	}
	init_exp(f, VCALL, luaK_codeABC(fs, OP_CALL, base, nparams+1, 2));
	luaK_fixline(fs, line);
	fs->freereg = base+1;  /* call remove function and arguments and leaves
						   (unless changed) one result */
}




/*
** {======================================================================
** Expression parsing
** =======================================================================
*/


static void prefixexp (LexState *ls, expdesc *v) {
	/* prefixexp -> NAME | '(' expr ')' */
	switch (ls->t.token) {
	case '(': {
		int line = ls->linenumber;
		luaX_next(ls);
		expr(ls, v);
		check_match(ls, ')', '(', line);
		luaK_dischargevars(ls->fs, v);
		return;
			  }
	case TK_NAME: {
		singlevar(ls, v);
		return;
				  }
	default: {
		luaX_syntaxerror(ls, "unexpected symbol");
		return;
			 }
	}
}


static void primaryexp (LexState *ls, expdesc *v) {
	/* primaryexp ->
	prefixexp { `.' NAME | `[' exp `]' | `:' NAME funcargs | funcargs } */
	FuncState *fs = ls->fs;
	prefixexp(ls, v);
	for (;;) {
		switch (ls->t.token) {
	  case '.': {  /* field */
		  field(ls, v);
		  break;
				}
	  case '[': {  /* `[' exp1 `]' */
		  expdesc key;
		  luaK_exp2anyreg(fs, v);
		  yindex(ls, &key);
		  luaK_indexed(fs, v, &key);
		  break;
				}
	  case ':': {  /* `:' NAME funcargs */
		  expdesc key;
		  luaX_next(ls);
		  checkname(ls, &key);
		  luaK_self(fs, v, &key);
		  funcargs(ls, v);
		  break;
				}
	  case '(': case TK_STRING: case '{': {  /* funcargs */
		  luaK_exp2nextreg(fs, v);
		  funcargs(ls, v);
		  break;
				}
	  default: return;
		}
	}
}


static void simpleexp (LexState *ls, expdesc *v) {
	/* simpleexp -> NUMBER | STRING | NIL | true | false | ... |
	constructor | FUNCTION body | primaryexp */
	switch (ls->t.token) {
	case TK_NUMBER: {
		init_exp(v, VKNUM, 0);
		v->u.nval = ls->t.seminfo.r;
		break;
					}
	case TK_STRING: {
		codestring(ls, v, ls->t.seminfo.ts);
		break;
					}
	case TK_NIL: {
		init_exp(v, VNIL, 0);
		break;
				 }
	case TK_TRUE: {
		init_exp(v, VTRUE, 0);
		break;
				  }
	case TK_FALSE: {
		init_exp(v, VFALSE, 0);
		break;
				   }
	case TK_DOTS: {  /* vararg */
		FuncState *fs = ls->fs;
		check_condition(ls, fs->f->is_vararg,
			"cannot use " LUA_QL("...") " outside a vararg function");
		fs->f->is_vararg &= ~VARARG_NEEDSARG;  /* don't need 'arg' */
		init_exp(v, VVARARG, luaK_codeABC(fs, OP_VARARG, 0, 1, 0));
		break;
				  }
	case '{': {  /* constructor */
		constructor(ls, v);
		return;
			  }
	case TK_FUNCTION: {
		luaX_next(ls);
		body(ls, v, 0, ls->linenumber);
		return;
					  }
	default: {
		primaryexp(ls, v);
		return;
			 }
	}
	luaX_next(ls);
}


static UnOpr getunopr (int op) {
	switch (op) {
	case TK_NOT: return OPR_NOT;
	case '-': return OPR_MINUS;
	case '#': return OPR_LEN;
	default: return OPR_NOUNOPR;
	}
}


static BinOpr getbinopr (int op) {
	switch (op) {
	case '+': return OPR_ADD;
	case '-': return OPR_SUB;
	case '*': return OPR_MUL;
	case '/': return OPR_DIV;
	case '%': return OPR_MOD;
	case '^': return OPR_POW;
	case TK_CONCAT: return OPR_CONCAT;
	case TK_NE: return OPR_NE;
	case TK_EQ: return OPR_EQ;
	case '<': return OPR_LT;
	case TK_LE: return OPR_LE;
	case '>': return OPR_GT;
	case TK_GE: return OPR_GE;
	case TK_AND: return OPR_AND;
	case TK_OR: return OPR_OR;
	default: return OPR_NOBINOPR;
	}
}


static const struct {
	lu_byte left;  /* left priority for each binary operator */
	lu_byte right; /* right priority */
} priority[] = {  /* ORDER OPR */
	{6, 6}, {6, 6}, {7, 7}, {7, 7}, {7, 7},  /* `+' `-' `/' `%' */
	{10, 9}, {5, 4},                 /* power and concat (right associative) */
	{3, 3}, {3, 3},                  /* equality and inequality */
	{3, 3}, {3, 3}, {3, 3}, {3, 3},  /* order */
	{2, 2}, {1, 1}                   /* logical (and/or) */
};

#define UNARY_PRIORITY	8  /* priority for unary operators */


/*
** subexpr -> (simpleexp | unop subexpr) { binop subexpr }
** where `binop' is any binary operator with a priority higher than `limit'
*/
static BinOpr subexpr (LexState *ls, expdesc *v, unsigned int limit) {
	BinOpr op;
	UnOpr uop;
	enterlevel(ls);
	uop = getunopr(ls->t.token);
	if (uop != OPR_NOUNOPR) {
		luaX_next(ls);
		subexpr(ls, v, UNARY_PRIORITY);
		luaK_prefix(ls->fs, uop, v);
	}
	else simpleexp(ls, v);
	/* expand while operators have priorities higher than `limit' */
	op = getbinopr(ls->t.token);
	while (op != OPR_NOBINOPR && priority[op].left > limit) {
		expdesc v2;
		BinOpr nextop;
		luaX_next(ls);
		luaK_infix(ls->fs, op, v);
		/* read sub-expression with higher priority */
		nextop = subexpr(ls, &v2, priority[op].right);
		luaK_posfix(ls->fs, op, v, &v2);
		op = nextop;
	}
	leavelevel(ls);
	return op;  /* return first untreated operator */
}


static void expr (LexState *ls, expdesc *v) {
	subexpr(ls, v, 0);
}

/* }==================================================================== */



/*
** {======================================================================
** Rules for Statements
** =======================================================================
*/


static int block_follow (int token) {
	switch (token) {
	case TK_ELSE: case TK_ELSEIF: case TK_END:
	case TK_UNTIL: case TK_EOS:
		return 1;
	default: return 0;
	}
}


static void block (LexState *ls) {
	/* block -> chunk */
	FuncState *fs = ls->fs;
	BlockCnt bl;
	enterblock(fs, &bl, 0);
	chunk(ls);
	lua_assert(bl.breaklist == NO_JUMP);
	leaveblock(fs);
}


/*
** structure to chain all variables in the left-hand side of an
** assignment
*/
struct LHS_assign {
	struct LHS_assign *prev;
	expdesc v;  /* variable (global, local, upvalue, or indexed) */
};


/*
** check whether, in an assignment to a local variable, the local variable
** is needed in a previous assignment (to a table). If so, save original
** local value in a safe place and use this safe copy in the previous
** assignment.
*/
static void check_conflict (LexState *ls, struct LHS_assign *lh, expdesc *v) {
	FuncState *fs = ls->fs;
	int extra = fs->freereg;  /* eventual position to save local variable */
	int conflict = 0;
	for (; lh; lh = lh->prev) {
		if (lh->v.k == VINDEXED) {
			if (lh->v.u.s.info == v->u.s.info) {  /* conflict? */
				conflict = 1;
				lh->v.u.s.info = extra;  /* previous assignment will use safe copy */
			}
			if (lh->v.u.s.aux == v->u.s.info) {  /* conflict? */
				conflict = 1;
				lh->v.u.s.aux = extra;  /* previous assignment will use safe copy */
			}
		}
	}
	if (conflict) {
		luaK_codeABC(fs, OP_MOVE, fs->freereg, v->u.s.info, 0);  /* make copy */
		luaK_reserveregs(fs, 1);
	}
}


static void assignment (LexState *ls, struct LHS_assign *lh, int nvars) {
	expdesc e;
	check_condition(ls, VLOCAL <= lh->v.k && lh->v.k <= VINDEXED,
		"syntax error");
	if (testnext(ls, ',')) {  /* assignment -> `,' primaryexp assignment */
		struct LHS_assign nv;
		nv.prev = lh;
		primaryexp(ls, &nv.v);
		if (nv.v.k == VLOCAL)
			check_conflict(ls, lh, &nv.v);
		assignment(ls, &nv, nvars+1);
	}
	else {  /* assignment -> `=' explist1 */
		int nexps;
		checknext(ls, '=');
		nexps = explist1(ls, &e);
		if (nexps != nvars) {
			adjust_assign(ls, nvars, nexps, &e);
			if (nexps > nvars)
				ls->fs->freereg -= nexps - nvars;  /* remove extra values */
		}
		else {
			luaK_setoneret(ls->fs, &e);  /* close last expression */
			luaK_storevar(ls->fs, &lh->v, &e);
			return;  /* avoid default */
		}
	}
	init_exp(&e, VNONRELOC, ls->fs->freereg-1);  /* default assignment */
	luaK_storevar(ls->fs, &lh->v, &e);
}


static int cond (LexState *ls) {
	/* cond -> exp */
	expdesc v;
	expr(ls, &v);  /* read condition */
	if (v.k == VNIL) v.k = VFALSE;  /* `falses' are all equal here */
	luaK_goiftrue(ls->fs, &v);
	return v.f;
}


static void breakstat (LexState *ls) {
	FuncState *fs = ls->fs;
	BlockCnt *bl = fs->bl;
	int upval = 0;
	while (bl && !bl->isbreakable) {
		upval |= bl->upval;
		bl = bl->previous;
	}
	if (!bl)
		luaX_syntaxerror(ls, "no loop to break");
	if (upval)
		luaK_codeABC(fs, OP_CLOSE, bl->nactvar, 0, 0);
	luaK_concat(fs, &bl->breaklist, luaK_jump(fs));
}


static void whilestat (LexState *ls, int line) {
	/* whilestat -> WHILE cond DO block END */
	FuncState *fs = ls->fs;
	int whileinit;
	int condexit;
	BlockCnt bl;
	luaX_next(ls);  /* skip WHILE */
	whileinit = luaK_getlabel(fs);
	condexit = cond(ls);
	enterblock(fs, &bl, 1);
	checknext(ls, TK_DO);
	block(ls);
	luaK_patchlist(fs, luaK_jump(fs), whileinit);
	check_match(ls, TK_END, TK_WHILE, line);
	leaveblock(fs);
	luaK_patchtohere(fs, condexit);  /* false conditions finish the loop */
}


static void repeatstat (LexState *ls, int line) {
	/* repeatstat -> REPEAT block UNTIL cond */
	int condexit;
	FuncState *fs = ls->fs;
	int repeat_init = luaK_getlabel(fs);
	BlockCnt bl1, bl2;
	enterblock(fs, &bl1, 1);  /* loop block */
	enterblock(fs, &bl2, 0);  /* scope block */
	luaX_next(ls);  /* skip REPEAT */
	chunk(ls);
	check_match(ls, TK_UNTIL, TK_REPEAT, line);
	condexit = cond(ls);  /* read condition (inside scope block) */
	if (!bl2.upval) {  /* no upvalues? */
		leaveblock(fs);  /* finish scope */
		luaK_patchlist(ls->fs, condexit, repeat_init);  /* close the loop */
	}
	else {  /* complete semantics when there are upvalues */
		breakstat(ls);  /* if condition then break */
		luaK_patchtohere(ls->fs, condexit);  /* else... */
		leaveblock(fs);  /* finish scope... */
		luaK_patchlist(ls->fs, luaK_jump(fs), repeat_init);  /* and repeat */
	}
	leaveblock(fs);  /* finish loop */
}


static int exp1 (LexState *ls) {
	expdesc e;
	int k;
	expr(ls, &e);
	k = e.k;
	luaK_exp2nextreg(ls->fs, &e);
	return k;
}


static void forbody (LexState *ls, int base, int line, int nvars, int isnum) {
	/* forbody -> DO block */
	BlockCnt bl;
	FuncState *fs = ls->fs;
	int prep, endfor;
	adjustlocalvars(ls, 3);  /* control variables */
	checknext(ls, TK_DO);
	prep = isnum ? luaK_codeAsBx(fs, OP_FORPREP, base, NO_JUMP) : luaK_jump(fs);
	enterblock(fs, &bl, 0);  /* scope for declared variables */
	adjustlocalvars(ls, nvars);
	luaK_reserveregs(fs, nvars);
	block(ls);
	leaveblock(fs);  /* end of scope for declared variables */
	luaK_patchtohere(fs, prep);
	endfor = (isnum) ? luaK_codeAsBx(fs, OP_FORLOOP, base, NO_JUMP) :
		luaK_codeABC(fs, OP_TFORLOOP, base, 0, nvars);
	luaK_fixline(fs, line);  /* pretend that `OP_FOR' starts the loop */
	luaK_patchlist(fs, (isnum ? endfor : luaK_jump(fs)), prep + 1);
}


static void fornum (LexState *ls, TString *varname, int line) {
	/* fornum -> NAME = exp1,exp1[,exp1] forbody */
	FuncState *fs = ls->fs;
	int base = fs->freereg;
	new_localvarliteral(ls, "(for index)", 0);
	new_localvarliteral(ls, "(for limit)", 1);
	new_localvarliteral(ls, "(for step)", 2);
	new_localvar(ls, varname, 3);
	checknext(ls, '=');
	exp1(ls);  /* initial value */
	checknext(ls, ',');
	exp1(ls);  /* limit */
	if (testnext(ls, ','))
		exp1(ls);  /* optional step */
	else {  /* default step = 1 */
		luaK_codeABx(fs, OP_LOADK, fs->freereg, luaK_numberK(fs, 1));
		luaK_reserveregs(fs, 1);
	}
	forbody(ls, base, line, 1, 1);
}


static void forlist (LexState *ls, TString *indexname) {
	/* forlist -> NAME {,NAME} IN explist1 forbody */
	FuncState *fs = ls->fs;
	expdesc e;
	int nvars = 0;
	int line;
	int base = fs->freereg;
	/* create control variables */
	new_localvarliteral(ls, "(for generator)", nvars++);
	new_localvarliteral(ls, "(for state)", nvars++);
	new_localvarliteral(ls, "(for control)", nvars++);
	/* create declared variables */
	new_localvar(ls, indexname, nvars++);
	while (testnext(ls, ','))
		new_localvar(ls, str_checkname(ls), nvars++);
	checknext(ls, TK_IN);
	line = ls->linenumber;
	adjust_assign(ls, 3, explist1(ls, &e), &e);
	luaK_checkstack(fs, 3);  /* extra space to call generator */
	forbody(ls, base, line, nvars - 3, 0);
}


static void forstat (LexState *ls, int line) {
	/* forstat -> FOR (fornum | forlist) END */
	FuncState *fs = ls->fs;
	TString *varname;
	BlockCnt bl;
	enterblock(fs, &bl, 1);  /* scope for loop and control variables */
	luaX_next(ls);  /* skip `for' */
	varname = str_checkname(ls);  /* first variable name */
	switch (ls->t.token) {
	case '=': fornum(ls, varname, line); break;
	case ',': case TK_IN: forlist(ls, varname); break;
	default: luaX_syntaxerror(ls, LUA_QL("=") " or " LUA_QL("in") " expected");
	}
	check_match(ls, TK_END, TK_FOR, line);
	leaveblock(fs);  /* loop scope (`break' jumps to this point) */
}


static int test_then_block (LexState *ls) {
	/* test_then_block -> [IF | ELSEIF] cond THEN block */
	int condexit;
	luaX_next(ls);  /* skip IF or ELSEIF */
	condexit = cond(ls);
	checknext(ls, TK_THEN);
	block(ls);  /* `then' part */
	return condexit;
}


static void ifstat (LexState *ls, int line) {
	/* ifstat -> IF cond THEN block {ELSEIF cond THEN block} [ELSE block] END */
	FuncState *fs = ls->fs;
	int flist;
	int escapelist = NO_JUMP;
	flist = test_then_block(ls);  /* IF cond THEN block */
	while (ls->t.token == TK_ELSEIF) {
		luaK_concat(fs, &escapelist, luaK_jump(fs));
		luaK_patchtohere(fs, flist);
		flist = test_then_block(ls);  /* ELSEIF cond THEN block */
	}
	if (ls->t.token == TK_ELSE) {
		luaK_concat(fs, &escapelist, luaK_jump(fs));
		luaK_patchtohere(fs, flist);
		luaX_next(ls);  /* skip ELSE (after patch, for correct line info) */
		block(ls);  /* `else' part */
	}
	else
		luaK_concat(fs, &escapelist, flist);
	luaK_patchtohere(fs, escapelist);
	check_match(ls, TK_END, TK_IF, line);
}


static void localfunc (LexState *ls) {
	expdesc v, b;
	FuncState *fs = ls->fs;
	new_localvar(ls, str_checkname(ls), 0);
	init_exp(&v, VLOCAL, fs->freereg);
	luaK_reserveregs(fs, 1);
	adjustlocalvars(ls, 1);
	body(ls, &b, 0, ls->linenumber);
	luaK_storevar(fs, &v, &b);
	/* debug information will only see the variable after this point! */
	getlocvar(fs, fs->nactvar - 1).startpc = fs->pc;
}


static void localstat (LexState *ls) {
	/* stat -> LOCAL NAME {`,' NAME} [`=' explist1] */
	int nvars = 0;
	int nexps;
	expdesc e;
	do {
		new_localvar(ls, str_checkname(ls), nvars++);
	} while (testnext(ls, ','));
	if (testnext(ls, '='))
		nexps = explist1(ls, &e);
	else {
		e.k = VVOID;
		nexps = 0;
	}
	adjust_assign(ls, nvars, nexps, &e);
	adjustlocalvars(ls, nvars);
}


static int funcname (LexState *ls, expdesc *v) {
	/* funcname -> NAME {field} [`:' NAME] */
	int needself = 0;
	singlevar(ls, v);
	while (ls->t.token == '.')
		field(ls, v);
	if (ls->t.token == ':') {
		needself = 1;
		field(ls, v);
	}
	return needself;
}


static void funcstat (LexState *ls, int line) {
	/* funcstat -> FUNCTION funcname body */
	int needself;
	expdesc v, b;
	luaX_next(ls);  /* skip FUNCTION */
	needself = funcname(ls, &v);
	body(ls, &b, needself, line);
	luaK_storevar(ls->fs, &v, &b);
	luaK_fixline(ls->fs, line);  /* definition `happens' in the first line */
}


static void exprstat (LexState *ls) {
	/* stat -> func | assignment */
	FuncState *fs = ls->fs;
	struct LHS_assign v;
	primaryexp(ls, &v.v);
	if (v.v.k == VCALL)  /* stat -> func */
		SETARG_C(getcode(fs, &v.v), 1);  /* call statement uses no results */
	else {  /* stat -> assignment */
		v.prev = NULL;
		assignment(ls, &v, 1);
	}
}


static void retstat (LexState *ls) {
	/* stat -> RETURN explist */
	FuncState *fs = ls->fs;
	expdesc e;
	int first, nret;  /* registers with returned values */
	luaX_next(ls);  /* skip RETURN */
	if (block_follow(ls->t.token) || ls->t.token == ';')
		first = nret = 0;  /* return no values */
	else {
		nret = explist1(ls, &e);  /* optional return values */
		if (hasmultret(e.k)) {
			luaK_setmultret(fs, &e);
			if (e.k == VCALL && nret == 1) {  /* tail call? */
				SET_OPCODE(getcode(fs,&e), OP_TAILCALL);
				lua_assert(GETARG_A(getcode(fs,&e)) == fs->nactvar);
			}
			first = fs->nactvar;
			nret = LUA_MULTRET;  /* return all values */
		}
		else {
			if (nret == 1)  /* only one single value? */
				first = luaK_exp2anyreg(fs, &e);
			else {
				luaK_exp2nextreg(fs, &e);  /* values must go to the `stack' */
				first = fs->nactvar;  /* return all `active' values */
				lua_assert(nret == fs->freereg - first);
			}
		}
	}
	luaK_ret(fs, first, nret);
}


static int statement (LexState *ls) {
	int line = ls->linenumber;  /* may be needed for error messages */
	switch (ls->t.token) {
	case TK_IF: {  /* stat -> ifstat */
		ifstat(ls, line);
		return 0;
				}
	case TK_WHILE: {  /* stat -> whilestat */
		whilestat(ls, line);
		return 0;
				   }
	case TK_DO: {  /* stat -> DO block END */
		luaX_next(ls);  /* skip DO */
		block(ls);
		check_match(ls, TK_END, TK_DO, line);
		return 0;
				}
	case TK_FOR: {  /* stat -> forstat */
		forstat(ls, line);
		return 0;
				 }
	case TK_REPEAT: {  /* stat -> repeatstat */
		repeatstat(ls, line);
		return 0;
					}
	case TK_FUNCTION: {
		funcstat(ls, line);  /* stat -> funcstat */
		return 0;
					  }
	case TK_LOCAL: {  /* stat -> localstat */
		luaX_next(ls);  /* skip LOCAL */
		if (testnext(ls, TK_FUNCTION))  /* local function? */
			localfunc(ls);
		else
			localstat(ls);
		return 0;
				   }
	case TK_RETURN: {  /* stat -> retstat */
		retstat(ls);
		return 1;  /* must be last statement */
					}
	case TK_BREAK: {  /* stat -> breakstat */
		luaX_next(ls);  /* skip BREAK */
		breakstat(ls);
		return 1;  /* must be last statement */
				   }
	default: {
		exprstat(ls);
		return 0;  /* to avoid warnings */
			 }
	}
}


static void chunk (LexState *ls) {
	/* chunk -> { stat [`;'] } */
	int islast = 0;
	enterlevel(ls);
	while (!islast && !block_follow(ls->t.token)) {
		islast = statement(ls);
		testnext(ls, ';');
		lua_assert(ls->fs->f->maxstacksize >= ls->fs->freereg &&
			ls->fs->freereg >= ls->fs->nactvar);
		ls->fs->freereg = ls->fs->nactvar;  /* free registers */
	}
	leavelevel(ls);
}

/* }====================================================================== */

//-------------------------------------------------------------lstate.c-----------------------------------------------------
#define state_size(x)	(sizeof(x) + LUAI_EXTRASPACE)
#define fromstate(l)	(cast(lu_byte *, (l)) - LUAI_EXTRASPACE)
#define tostate(l)   (cast(lua_State *, cast(lu_byte *, l) + LUAI_EXTRASPACE))


/*
** Main thread combines a thread state and the global state
*/
typedef struct LG {
	lua_State l;
	global_State g;
} LG;

static void stack_init (lua_State *L1, lua_State *L) {
	/* initialize CallInfo array */
	L1->base_ci = luaM_newvector(L, BASIC_CI_SIZE, CallInfo);
	L1->ci = L1->base_ci;
	L1->size_ci = BASIC_CI_SIZE;
	L1->end_ci = L1->base_ci + L1->size_ci - 1;
	/* initialize stack array */
	L1->stack = luaM_newvector(L, BASIC_STACK_SIZE + EXTRA_STACK, TValue);
	L1->stacksize = BASIC_STACK_SIZE + EXTRA_STACK;
	L1->top = L1->stack;
	L1->stack_last = L1->stack+(L1->stacksize - EXTRA_STACK)-1;
	/* initialize first ci */
	L1->ci->func = L1->top;
	setnilvalue(L1->top++);  /* `function' entry for this `ci' */
	L1->base = L1->ci->base = L1->top;
	L1->ci->top = L1->top + LUA_MINSTACK;
}


static void freestack (lua_State *L, lua_State *L1) {
	luaM_freearray(L, L1->base_ci, L1->size_ci, CallInfo);
	luaM_freearray(L, L1->stack, L1->stacksize, TValue);
}


/*
** open parts that may cause memory-allocation errors
*/
static void f_luaopen (lua_State *L, void *ud) {
	global_State *g = G(L);
	UNUSED(ud);
	stack_init(L, L);  /* init stack */
	sethvalue(L, gt(L), luaH_new(L, 0, 2));  /* table of globals */
	sethvalue(L, registry(L), luaH_new(L, 0, 2));  /* registry */
	luaS_resize(L, MINSTRTABSIZE);  /* initial size of string table */
	luaT_init(L);
	luaX_init(L);
	luaS_fix(luaS_newliteral(L, MEMERRMSG));
	g->GCthreshold = 4*g->totalbytes;
}


static void preinit_state (lua_State *L, global_State *g) {
	G(L) = g;
	L->stack = NULL;
	L->stacksize = 0;
	L->errorJmp = NULL;
	L->hook = NULL;
	L->hookmask = 0;
	L->basehookcount = 0;
	L->allowhook = 1;
	resethookcount(L);
	L->openupval = NULL;
	L->size_ci = 0;
	L->nCcalls = 0;
	L->status = 0;
	L->base_ci = L->ci = NULL;
	L->savedpc = NULL;
	L->errfunc = 0;
	setnilvalue(gt(L));
}


static void close_state (lua_State *L) {
	global_State *g = G(L);
	luaF_close(L, L->stack);  /* close all upvalues for this thread */
	luaC_freeall(L);  /* collect all objects */
	lua_assert(g->rootgc == obj2gco(L));
	lua_assert(g->strt.nuse == 0);
	luaM_freearray(L, G(L)->strt.hash, G(L)->strt.size, TString *);
	luaZ_freebuffer(L, &g->buff);
	freestack(L, L);
	lua_assert(g->totalbytes == sizeof(LG));
	(*g->frealloc)(g->ud, fromstate(L), state_size(LG), 0);
}


lua_State *luaE_newthread (lua_State *L) {
	lua_State *L1 = tostate(luaM_malloc(L, state_size(lua_State)));
	luaC_link(L, obj2gco(L1), LUA_TTHREAD);
	preinit_state(L1, G(L));
	stack_init(L1, L);  /* init stack */
	setobj2n(L, gt(L1), gt(L));  /* share table of globals */
	L1->hookmask = L->hookmask;
	L1->basehookcount = L->basehookcount;
	L1->hook = L->hook;
	resethookcount(L1);
	lua_assert(iswhite(obj2gco(L1)));
	return L1;
}


void luaE_freethread (lua_State *L, lua_State *L1) {
	luaF_close(L1, L1->stack);  /* close all upvalues for this thread */
	lua_assert(L1->openupval == NULL);
	luai_userstatefree(L1);
	freestack(L, L1);
	luaM_freemem(L, fromstate(L1), state_size(lua_State));
}


LUA_API lua_State *lua_newstate (lua_Alloc f, void *ud) {
	int i;
	lua_State *L;
	global_State *g;
	void *l = (*f)(ud, NULL, 0, state_size(LG));
	if (l == NULL) return NULL;
	L = tostate(l);
	g = &((LG *)L)->g;
	L->next = NULL;
	L->tt = LUA_TTHREAD;
	g->currentwhite = bit2mask(WHITE0BIT, FIXEDBIT);
	L->marked = luaC_white(g);
	set2bits(L->marked, FIXEDBIT, SFIXEDBIT);
	preinit_state(L, g);
	g->frealloc = f;
	g->ud = ud;
	g->mainthread = L;
	g->uvhead.u.l.prev = &g->uvhead;
	g->uvhead.u.l.next = &g->uvhead;
	g->GCthreshold = 0;  /* mark it as unfinished state */
	g->strt.size = 0;
	g->strt.nuse = 0;
	g->strt.hash = NULL;
	setnilvalue(registry(L));
	luaZ_initbuffer(L, &g->buff);
	g->panic = NULL;
	g->gcstate = GCSpause;
	g->rootgc = obj2gco(L);
	g->sweepstrgc = 0;
	g->sweepgc = &g->rootgc;
	g->gray = NULL;
	g->grayagain = NULL;
	g->weak = NULL;
	g->tmudata = NULL;
	g->totalbytes = sizeof(LG);
	g->gcpause = LUAI_GCPAUSE;
	g->gcstepmul = LUAI_GCMUL;
	g->gcdept = 0;
	for (i=0; i<NUM_TAGS; i++) g->mt[i] = NULL;
	if (luaD_rawrunprotected(L, f_luaopen, NULL) != 0) {
		/* memory allocation error: free partial state */
		close_state(L);
		L = NULL;
	}
	else
		luai_userstateopen(L);
	return L;
}


static void callallgcTM (lua_State *L, void *ud) {
	UNUSED(ud);
	luaC_callGCTM(L);  /* call GC metamethods for all udata */
}


LUA_API void lua_close (lua_State *L) {
	L = G(L)->mainthread;  /* only the main thread can be closed */
	lua_lock(L);
	luaF_close(L, L->stack);  /* close all upvalues for this thread */
	luaC_separateudata(L, 1);  /* separate udata that have GC metamethods */
	L->errfunc = 0;  /* no error function during GC metamethods */
	do {  /* repeat until no more errors */
		L->ci = L->base_ci;
		L->base = L->top = L->ci->base;
		L->nCcalls = 0;
	} while (luaD_rawrunprotected(L, callallgcTM, NULL) != 0);
	lua_assert(G(L)->tmudata == NULL);
	luai_userstateclose(L);
	close_state(L);
}


//-------------------------------------------------------------lauxlib.c----------------------------------------------------
#define FREELIST_REF	0	/* free list of references */

/* convert a stack index to positive */
#define abs_index(L, i)		((i) > 0 || (i) <= LUA_REGISTRYINDEX ? (i) : \
	lua_gettop(L) + (i) + 1)


/*
** {======================================================
** Error-report functions
** =======================================================
*/

LUALIB_API int luaL_argerror (lua_State *L, int narg, const char *extramsg) {
	lua_Debug ar;
	if (!lua_getstack(L, 0, &ar))  /* no stack frame? */
		return luaL_error(L, "bad argument #%d (%s)", narg, extramsg);
	lua_getinfo(L, "n", &ar);
	if (strcmp(ar.namewhat, "method") == 0) {
		narg--;  /* do not count `self' */
		if (narg == 0)  /* error is in the self argument itself? */
			return luaL_error(L, "calling " LUA_QS " on bad self (%s)",
			ar.name, extramsg);
	}
	if (ar.name == NULL)
		ar.name = "?";
	return luaL_error(L, "bad argument #%d to " LUA_QS " (%s)",
		narg, ar.name, extramsg);
}


LUALIB_API int luaL_typerror (lua_State *L, int narg, const char *tname) {
	const char *msg = lua_pushfstring(L, "%s expected, got %s",
		tname, luaL_typename(L, narg));
	return luaL_argerror(L, narg, msg);
}


static void tag_error (lua_State *L, int narg, int tag) {
	luaL_typerror(L, narg, lua_typename(L, tag));
}


LUALIB_API void luaL_where (lua_State *L, int level) {
	lua_Debug ar;
	if (lua_getstack(L, level, &ar)) {  /* check function at level */
		lua_getinfo(L, "Sl", &ar);  /* get info about it */
		if (ar.currentline > 0) {  /* is there info? */
			lua_pushfstring(L, "%s:%d: ", ar.short_src, ar.currentline);
			return;
		}
	}
	lua_pushliteral(L, "");  /* else, no information available... */
}


LUALIB_API int luaL_error (lua_State *L, const char *fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	luaL_where(L, 1);
	lua_pushvfstring(L, fmt, argp);
	va_end(argp);
	lua_concat(L, 2);
	return lua_error(L);
}

/* }====================================================== */


LUALIB_API int luaL_checkoption (lua_State *L, int narg, const char *def,
								 const char *const lst[]) {
									 const char *name = (def) ? luaL_optstring(L, narg, def) :
										 luaL_checkstring(L, narg);
									 int i;
									 for (i=0; lst[i]; i++)
										 if (strcmp(lst[i], name) == 0)
											 return i;
									 return luaL_argerror(L, narg,
										 lua_pushfstring(L, "invalid option " LUA_QS, name));
}


LUALIB_API int luaL_newmetatable (lua_State *L, const char *tname) {
	lua_getfield(L, LUA_REGISTRYINDEX, tname);  /* get registry.name */
	if (!lua_isnil(L, -1))  /* name already in use? */
		return 0;  /* leave previous value on top, but return 0 */
	lua_pop(L, 1);
	lua_newtable(L);  /* create metatable */
	lua_pushvalue(L, -1);
	lua_setfield(L, LUA_REGISTRYINDEX, tname);  /* registry.name = metatable */
	return 1;
}


LUALIB_API void *luaL_checkudata (lua_State *L, int ud, const char *tname) {
	void *p = lua_touserdata(L, ud);
	if (p != NULL) {  /* value is a userdata? */
		if (lua_getmetatable(L, ud)) {  /* does it have a metatable? */
			lua_getfield(L, LUA_REGISTRYINDEX, tname);  /* get correct metatable */
			if (lua_rawequal(L, -1, -2)) {  /* does it have the correct mt? */
				lua_pop(L, 2);  /* remove both metatables */
				return p;
			}
		}
	}
	luaL_typerror(L, ud, tname);  /* else error */
	return NULL;  /* to avoid warnings */
}


LUALIB_API void luaL_checkstack (lua_State *L, int space, const char *mes) {
	if (!lua_checkstack(L, space))
		luaL_error(L, "stack overflow (%s)", mes);
}


LUALIB_API void luaL_checktype (lua_State *L, int narg, int t) {
	if (lua_type(L, narg) != t)
		tag_error(L, narg, t);
}


LUALIB_API void luaL_checkany (lua_State *L, int narg) {
	if (lua_type(L, narg) == LUA_TNONE)
		luaL_argerror(L, narg, "value expected");
}


LUALIB_API const char *luaL_checklstring (lua_State *L, int narg, size_t *len) {
	const char *s = lua_tolstring(L, narg, len);
	if (!s) tag_error(L, narg, LUA_TSTRING);
	return s;
}


LUALIB_API const char *luaL_optlstring (lua_State *L, int narg,
										const char *def, size_t *len) {
											if (lua_isnoneornil(L, narg)) {
												if (len)
													*len = (def ? strlen(def) : 0);
												return def;
											}
											else return luaL_checklstring(L, narg, len);
}


LUALIB_API lua_Number luaL_checknumber (lua_State *L, int narg) {
	lua_Number d = lua_tonumber(L, narg);
	if (d == 0 && !lua_isnumber(L, narg))  /* avoid extra test when d is not 0 */
		tag_error(L, narg, LUA_TNUMBER);
	return d;
}


LUALIB_API lua_Number luaL_optnumber (lua_State *L, int narg, lua_Number def) {
	return luaL_opt(L, luaL_checknumber, narg, def);
}


LUALIB_API lua_Integer luaL_checkinteger (lua_State *L, int narg) {
	lua_Integer d = lua_tointeger(L, narg);
	if (d == 0 && !lua_isnumber(L, narg))  /* avoid extra test when d is not 0 */
		tag_error(L, narg, LUA_TNUMBER);
	return d;
}


LUALIB_API lua_Integer luaL_optinteger (lua_State *L, int narg,
										lua_Integer def) {
											return luaL_opt(L, luaL_checkinteger, narg, def);
}


LUALIB_API int luaL_getmetafield (lua_State *L, int obj, const char *event) {
	if (!lua_getmetatable(L, obj))  /* no metatable? */
		return 0;
	lua_pushstring(L, event);
	lua_rawget(L, -2);
	if (lua_isnil(L, -1)) {
		lua_pop(L, 2);  /* remove metatable and metafield */
		return 0;
	}
	else {
		lua_remove(L, -2);  /* remove only metatable */
		return 1;
	}
}


LUALIB_API int luaL_callmeta (lua_State *L, int obj, const char *event) {
	obj = abs_index(L, obj);
	if (!luaL_getmetafield(L, obj, event))  /* no metafield? */
		return 0;
	lua_pushvalue(L, obj);
	lua_call(L, 1, 1);
	return 1;
}


LUALIB_API void (luaL_register) (lua_State *L, const char *libname,
								 const luaL_Reg *l) {
									 luaI_openlib(L, libname, l, 0);
}


static int libsize (const luaL_Reg *l) {
	int size = 0;
	for (; l->name; l++) size++;
	return size;
}


LUALIB_API void luaI_openlib (lua_State *L, const char *libname,
							  const luaL_Reg *l, int nup) {
								  if (libname) {
									  int size = libsize(l);
									  /* check whether lib already exists */
									  luaL_findtable(L, LUA_REGISTRYINDEX, "_LOADED", size);
									  lua_getfield(L, -1, libname);  /* get _LOADED[libname] */
									  if (!lua_istable(L, -1)) {  /* not found? */
										  lua_pop(L, 1);  /* remove previous result */
										  /* try global variable (and create one if it does not exist) */
										  if (luaL_findtable(L, LUA_GLOBALSINDEX, libname, size) != NULL)
											  luaL_error(L, "name conflict for module " LUA_QS, libname);
										  lua_pushvalue(L, -1);
										  lua_setfield(L, -3, libname);  /* _LOADED[libname] = new table */
									  }
									  lua_remove(L, -2);  /* remove _LOADED table */
									  lua_insert(L, -(nup+1));  /* move library table to below upvalues */
								  }
								  for (; l->name; l++) {
									  int i;
									  for (i=0; i<nup; i++)  /* copy upvalues to the top */
										  lua_pushvalue(L, -nup);
									  lua_pushcclosure(L, l->func, nup);
									  lua_setfield(L, -(nup+2), l->name);
								  }
								  lua_pop(L, nup);  /* remove upvalues */
}



/*
** {======================================================
** getn-setn: size for arrays
** =======================================================
*/

#if defined(LUA_COMPAT_GETN)

static int checkint (lua_State *L, int topop) {
	int n = (lua_type(L, -1) == LUA_TNUMBER) ? lua_tointeger(L, -1) : -1;
	lua_pop(L, topop);
	return n;
}


static void getsizes (lua_State *L) {
	lua_getfield(L, LUA_REGISTRYINDEX, "LUA_SIZES");
	if (lua_isnil(L, -1)) {  /* no `size' table? */
		lua_pop(L, 1);  /* remove nil */
		lua_newtable(L);  /* create it */
		lua_pushvalue(L, -1);  /* `size' will be its own metatable */
		lua_setmetatable(L, -2);
		lua_pushliteral(L, "kv");
		lua_setfield(L, -2, "__mode");  /* metatable(N).__mode = "kv" */
		lua_pushvalue(L, -1);
		lua_setfield(L, LUA_REGISTRYINDEX, "LUA_SIZES");  /* store in register */
	}
}


LUALIB_API void luaL_setn (lua_State *L, int t, int n) {
	t = abs_index(L, t);
	lua_pushliteral(L, "n");
	lua_rawget(L, t);
	if (checkint(L, 1) >= 0) {  /* is there a numeric field `n'? */
		lua_pushliteral(L, "n");  /* use it */
		lua_pushinteger(L, n);
		lua_rawset(L, t);
	}
	else {  /* use `sizes' */
		getsizes(L);
		lua_pushvalue(L, t);
		lua_pushinteger(L, n);
		lua_rawset(L, -3);  /* sizes[t] = n */
		lua_pop(L, 1);  /* remove `sizes' */
	}
}


LUALIB_API int luaL_getn (lua_State *L, int t) {
	int n;
	t = abs_index(L, t);
	lua_pushliteral(L, "n");  /* try t.n */
	lua_rawget(L, t);
	if ((n = checkint(L, 1)) >= 0) return n;
	getsizes(L);  /* else try sizes[t] */
	lua_pushvalue(L, t);
	lua_rawget(L, -2);
	if ((n = checkint(L, 2)) >= 0) return n;
	return (int)lua_objlen(L, t);
}

#endif

/* }====================================================== */



LUALIB_API const char *luaL_gsub (lua_State *L, const char *s, const char *p,
								  const char *r) {
									  const char *wild;
									  size_t l = strlen(p);
									  luaL_Buffer b;
									  luaL_buffinit(L, &b);
									  while ((wild = strstr(s, p)) != NULL) {
										  luaL_addlstring(&b, s, wild - s);  /* push prefix */
										  luaL_addstring(&b, r);  /* push replacement in place of pattern */
										  s = wild + l;  /* continue after `p' */
									  }
									  luaL_addstring(&b, s);  /* push last suffix */
									  luaL_pushresult(&b);
									  return lua_tostring(L, -1);
}


LUALIB_API const char *luaL_findtable (lua_State *L, int idx,
									   const char *fname, int szhint) {
										   const char *e;
										   lua_pushvalue(L, idx);
										   do {
											   e = strchr(fname, '.');
											   if (e == NULL) e = fname + strlen(fname);
											   lua_pushlstring(L, fname, e - fname);
											   lua_rawget(L, -2);
											   if (lua_isnil(L, -1)) {  /* no such field? */
												   lua_pop(L, 1);  /* remove this nil */
												   lua_createtable(L, 0, (*e == '.' ? 1 : szhint)); /* new table for field */
												   lua_pushlstring(L, fname, e - fname);
												   lua_pushvalue(L, -2);
												   lua_settable(L, -4);  /* set new table into field */
											   }
											   else if (!lua_istable(L, -1)) {  /* field has a non-table value? */
												   lua_pop(L, 2);  /* remove table and value */
												   return fname;  /* return problematic part of the name */
											   }
											   lua_remove(L, -2);  /* remove previous table */
											   fname = e + 1;
										   } while (*e == '.');
										   return NULL;
}



/*
** {======================================================
** Generic Buffer manipulation
** =======================================================
*/


#define bufflen(B)	((B)->p - (B)->buffer)
#define bufffree(B)	((size_t)(LUAL_BUFFERSIZE - bufflen(B)))

#define LIMIT	(LUA_MINSTACK/2)


static int emptybuffer (luaL_Buffer *B) {
	size_t l = bufflen(B);
	if (l == 0) return 0;  /* put nothing on stack */
	else {
		lua_pushlstring(B->L, B->buffer, l);
		B->p = B->buffer;
		B->lvl++;
		return 1;
	}
}


static void adjuststack (luaL_Buffer *B) {
	if (B->lvl > 1) {
		lua_State *L = B->L;
		int toget = 1;  /* number of levels to concat */
		size_t toplen = lua_strlen(L, -1);
		do {
			size_t l = lua_strlen(L, -(toget+1));
			if (B->lvl - toget + 1 >= LIMIT || toplen > l) {
				toplen += l;
				toget++;
			}
			else break;
		} while (toget < B->lvl);
		lua_concat(L, toget);
		B->lvl = B->lvl - toget + 1;
	}
}


LUALIB_API char *luaL_prepbuffer (luaL_Buffer *B) {
	if (emptybuffer(B))
		adjuststack(B);
	return B->buffer;
}


LUALIB_API void luaL_addlstring (luaL_Buffer *B, const char *s, size_t l) {
	while (l--)
		luaL_addchar(B, *s++);
}


LUALIB_API void luaL_addstring (luaL_Buffer *B, const char *s) {
	luaL_addlstring(B, s, strlen(s));
}


LUALIB_API void luaL_pushresult (luaL_Buffer *B) {
	emptybuffer(B);
	lua_concat(B->L, B->lvl);
	B->lvl = 1;
}


LUALIB_API void luaL_addvalue (luaL_Buffer *B) {
	lua_State *L = B->L;
	size_t vl;
	const char *s = lua_tolstring(L, -1, &vl);
	if (vl <= bufffree(B)) {  /* fit into buffer? */
		memcpy(B->p, s, vl);  /* put it there */
		B->p += vl;
		lua_pop(L, 1);  /* remove from stack */
	}
	else {
		if (emptybuffer(B))
			lua_insert(L, -2);  /* put buffer before new value */
		B->lvl++;  /* add new value into B stack */
		adjuststack(B);
	}
}


LUALIB_API void luaL_buffinit (lua_State *L, luaL_Buffer *B) {
	B->L = L;
	B->p = B->buffer;
	B->lvl = 0;
}

/* }====================================================== */


LUALIB_API int luaL_ref (lua_State *L, int t) {
	int ref;
	t = abs_index(L, t);
	if (lua_isnil(L, -1)) {
		lua_pop(L, 1);  /* remove from stack */
		return LUA_REFNIL;  /* `nil' has a unique fixed reference */
	}
	lua_rawgeti(L, t, FREELIST_REF);  /* get first free element */
	ref = (int)lua_tointeger(L, -1);  /* ref = t[FREELIST_REF] */
	lua_pop(L, 1);  /* remove it from stack */
	if (ref != 0) {  /* any free element? */
		lua_rawgeti(L, t, ref);  /* remove it from list */
		lua_rawseti(L, t, FREELIST_REF);  /* (t[FREELIST_REF] = t[ref]) */
	}
	else {  /* no free elements */
		ref = (int)lua_objlen(L, t);
		ref++;  /* create new reference */
	}
	lua_rawseti(L, t, ref);
	return ref;
}


LUALIB_API void luaL_unref (lua_State *L, int t, int ref) {
	if (ref >= 0) {
		t = abs_index(L, t);
		lua_rawgeti(L, t, FREELIST_REF);
		lua_rawseti(L, t, ref);  /* t[ref] = t[FREELIST_REF] */
		lua_pushinteger(L, ref);
		lua_rawseti(L, t, FREELIST_REF);  /* t[FREELIST_REF] = ref */
	}
}



/*
** {======================================================
** Load functions
** =======================================================
*/

typedef struct LoadF {
	int extraline;
	FILE *f;
	char buff[LUAL_BUFFERSIZE];
} LoadF;


static const char *getF (lua_State *L, void *ud, size_t *size) {
	LoadF *lf = (LoadF *)ud;
	(void)L;
	if (lf->extraline) {
		lf->extraline = 0;
		*size = 1;
		return "\n";
	}
	if (feof(lf->f)) return NULL;
	*size = fread(lf->buff, 1, LUAL_BUFFERSIZE, lf->f);
	return (*size > 0) ? lf->buff : NULL;
}


static int errfile (lua_State *L, const char *what, int fnameindex) {
	const char *serr = strerror(errno);
	const char *filename = lua_tostring(L, fnameindex) + 1;
	lua_pushfstring(L, "cannot %s %s: %s", what, filename, serr);
	lua_remove(L, fnameindex);
	return LUA_ERRFILE;
}


LUALIB_API int luaL_loadfile (lua_State *L, const char *filename) {
	LoadF lf;
	int status, readstatus;
	int c;
	int fnameindex = lua_gettop(L) + 1;  /* index of filename on the stack */
	lf.extraline = 0;
	if (filename == NULL) {
		lua_pushliteral(L, "=stdin");
		lf.f = stdin;
	}
	else {
		lua_pushfstring(L, "@%s", filename);
		lf.f = fopen(filename, "r");
		if (lf.f == NULL) return errfile(L, "open", fnameindex);
	}
	c = getc(lf.f);
	if (c == '#') {  /* Unix exec. file? */
		lf.extraline = 1;
		while ((c = getc(lf.f)) != EOF && c != '\n') ;  /* skip first line */
		if (c == '\n') c = getc(lf.f);
	}
	if (c == LUA_SIGNATURE[0] && lf.f != stdin) {  /* binary file? */
		fclose(lf.f);
		lf.f = fopen(filename, "rb");  /* reopen in binary mode */
		if (lf.f == NULL) return errfile(L, "reopen", fnameindex);
		/* skip eventual `#!...' */
		while ((c = getc(lf.f)) != EOF && c != LUA_SIGNATURE[0]) ;
		lf.extraline = 0;
	}
	ungetc(c, lf.f);
	status = lua_load(L, getF, &lf, lua_tostring(L, -1));
	readstatus = ferror(lf.f);
	if (lf.f != stdin) fclose(lf.f);  /* close file (even in case of errors) */
	if (readstatus) {
		lua_settop(L, fnameindex);  /* ignore results from `lua_load' */
		return errfile(L, "read", fnameindex);
	}
	lua_remove(L, fnameindex);
	return status;
}


typedef struct LoadS {
	const char *s;
	size_t size;
} LoadS;


static const char *getS (lua_State *L, void *ud, size_t *size) {
	LoadS *ls = (LoadS *)ud;
	(void)L;
	if (ls->size == 0) return NULL;
	*size = ls->size;
	ls->size = 0;
	return ls->s;
}


LUALIB_API int luaL_loadbuffer (lua_State *L, const char *buff, size_t size,
								const char *name) {
									LoadS ls;
									ls.s = buff;
									ls.size = size;
									return lua_load(L, getS, &ls, name);
}


LUALIB_API int (luaL_loadstring) (lua_State *L, const char *s) {
	return luaL_loadbuffer(L, s, strlen(s), s);
}



/* }====================================================== */


static void *l_alloc (void *ud, void *ptr, size_t osize, size_t nsize) {
	(void)ud;
	(void)osize;
	if (nsize == 0) {
		free(ptr);
		return NULL;
	}
	else
		return realloc(ptr, nsize);
}


static int panic (lua_State *L) {
	(void)L;  /* to avoid warnings */
	fprintf(stderr, "PANIC: unprotected error in call to Lua API (%s)\n",
		lua_tostring(L, -1));
	return 0;
}


LUALIB_API lua_State *luaL_newstate (void) {
	lua_State *L = lua_newstate(l_alloc, NULL);
	if (L) lua_atpanic(L, &panic);
	return L;
}


//-------------------------------------------------------------lmathlib.c--------------------------------------------------------

#include <stdlib.h>
#include <math.h>

//#define lmathlib_c
//#define LUA_LIB

//#include "lua.h"

//#include "lauxlib.h"
//#include "lualib.h"


#undef PI
#define PI (3.14159265358979323846)
#define RADIANS_PER_DEGREE (PI/180.0)



static int math_abs (lua_State *L) {
	lua_pushnumber(L, fabs(luaL_checknumber(L, 1)));
	return 1;
}

static int math_sin (lua_State *L) {
	lua_pushnumber(L, sin(luaL_checknumber(L, 1)));
	return 1;
}

static int math_sinh (lua_State *L) {
	lua_pushnumber(L, sinh(luaL_checknumber(L, 1)));
	return 1;
}

static int math_cos (lua_State *L) {
	lua_pushnumber(L, cos(luaL_checknumber(L, 1)));
	return 1;
}

static int math_cosh (lua_State *L) {
	lua_pushnumber(L, cosh(luaL_checknumber(L, 1)));
	return 1;
}

static int math_tan (lua_State *L) {
	lua_pushnumber(L, tan(luaL_checknumber(L, 1)));
	return 1;
}

static int math_tanh (lua_State *L) {
	lua_pushnumber(L, tanh(luaL_checknumber(L, 1)));
	return 1;
}

static int math_asin (lua_State *L) {
	lua_pushnumber(L, asin(luaL_checknumber(L, 1)));
	return 1;
}

static int math_acos (lua_State *L) {
	lua_pushnumber(L, acos(luaL_checknumber(L, 1)));
	return 1;
}

static int math_atan (lua_State *L) {
	lua_pushnumber(L, atan(luaL_checknumber(L, 1)));
	return 1;
}

static int math_atan2 (lua_State *L) {
	lua_pushnumber(L, atan2(luaL_checknumber(L, 1), luaL_checknumber(L, 2)));
	return 1;
}

static int math_ceil (lua_State *L) {
	lua_pushnumber(L, ceil(luaL_checknumber(L, 1)));
	return 1;
}

static int math_floor (lua_State *L) {
	lua_pushnumber(L, floor(luaL_checknumber(L, 1)));
	return 1;
}

static int math_fmod (lua_State *L) {
	lua_pushnumber(L, fmod(luaL_checknumber(L, 1), luaL_checknumber(L, 2)));
	return 1;
}

static int math_modf (lua_State *L) {
	double ip;
	double fp = modf(luaL_checknumber(L, 1), &ip);
	lua_pushnumber(L, ip);
	lua_pushnumber(L, fp);
	return 2;
}

static int math_sqrt (lua_State *L) {
	lua_pushnumber(L, sqrt(luaL_checknumber(L, 1)));
	return 1;
}

static int math_pow (lua_State *L) {
	lua_pushnumber(L, pow(luaL_checknumber(L, 1), luaL_checknumber(L, 2)));
	return 1;
}

static int math_log (lua_State *L) {
	lua_pushnumber(L, log(luaL_checknumber(L, 1)));
	return 1;
}

static int math_log10 (lua_State *L) {
	lua_pushnumber(L, log10(luaL_checknumber(L, 1)));
	return 1;
}

static int math_exp (lua_State *L) {
	lua_pushnumber(L, exp(luaL_checknumber(L, 1)));
	return 1;
}

static int math_deg (lua_State *L) {
	lua_pushnumber(L, luaL_checknumber(L, 1)/RADIANS_PER_DEGREE);
	return 1;
}

static int math_rad (lua_State *L) {
	lua_pushnumber(L, luaL_checknumber(L, 1)*RADIANS_PER_DEGREE);
	return 1;
}

static int math_frexp (lua_State *L) {
	int e;
	lua_pushnumber(L, frexp(luaL_checknumber(L, 1), &e));
	lua_pushinteger(L, e);
	return 2;
}

static int math_ldexp (lua_State *L) {
	lua_pushnumber(L, ldexp(luaL_checknumber(L, 1), luaL_checkint(L, 2)));
	return 1;
}



static int math_min (lua_State *L) {
	int n = lua_gettop(L);  /* number of arguments */
	lua_Number dmin = luaL_checknumber(L, 1);
	int i;
	for (i=2; i<=n; i++) {
		lua_Number d = luaL_checknumber(L, i);
		if (d < dmin)
			dmin = d;
	}
	lua_pushnumber(L, dmin);
	return 1;
}


static int math_max (lua_State *L) {
	int n = lua_gettop(L);  /* number of arguments */
	lua_Number dmax = luaL_checknumber(L, 1);
	int i;
	for (i=2; i<=n; i++) {
		lua_Number d = luaL_checknumber(L, i);
		if (d > dmax)
			dmax = d;
	}
	lua_pushnumber(L, dmax);
	return 1;
}


static int math_random (lua_State *L) {
	/* the `%' avoids the (rare) case of r==1, and is needed also because on
	some systems (SunOS!) `rand()' may return a value larger than RAND_MAX */
	lua_Number r = (lua_Number)(rand()%RAND_MAX) / (lua_Number)RAND_MAX;
	switch (lua_gettop(L)) {  /* check number of arguments */
	case 0: {  /* no arguments */
		lua_pushnumber(L, r);  /* Number between 0 and 1 */
		break;
			}
	case 1: {  /* only upper limit */
		int u = luaL_checkint(L, 1);
		luaL_argcheck(L, 1<=u, 1, "interval is empty");
		lua_pushnumber(L, floor(r*u)+1);  /* int between 1 and `u' */
		break;
			}
	case 2: {  /* lower and upper limits */
		int l = luaL_checkint(L, 1);
		int u = luaL_checkint(L, 2);
		luaL_argcheck(L, l<=u, 2, "interval is empty");
		lua_pushnumber(L, floor(r*(u-l+1))+l);  /* int between `l' and `u' */
		break;
			}
	default: return luaL_error(L, "wrong number of arguments");
	}
	return 1;
}


static int math_randomseed (lua_State *L) {
	srand(luaL_checkint(L, 1));
	return 0;
}


static const luaL_Reg mathlib[] = {
	{"abs",   math_abs},
	{"acos",  math_acos},
	{"asin",  math_asin},
	{"atan2", math_atan2},
	{"atan",  math_atan},
	{"ceil",  math_ceil},
	{"cosh",   math_cosh},
	{"cos",   math_cos},
	{"deg",   math_deg},
	{"exp",   math_exp},
	{"floor", math_floor},
	{"fmod",   math_fmod},
	{"frexp", math_frexp},
	{"ldexp", math_ldexp},
	{"log10", math_log10},
	{"log",   math_log},
	{"max",   math_max},
	{"min",   math_min},
	{"modf",   math_modf},
	{"pow",   math_pow},
	{"rad",   math_rad},
	{"random",     math_random},
	{"randomseed", math_randomseed},
	{"sinh",   math_sinh},
	{"sin",   math_sin},
	{"sqrt",  math_sqrt},
	{"tanh",   math_tanh},
	{"tan",   math_tan},
	{NULL, NULL}
};


/*
** Open math library
*/
LUALIB_API int luaopen_math (lua_State *L) {
	luaL_register(L, LUA_MATHLIBNAME, mathlib);
	lua_pushnumber(L, PI);
	lua_setfield(L, -2, "pi");
	lua_pushnumber(L, HUGE_VAL);
	lua_setfield(L, -2, "huge");
#if defined(LUA_COMPAT_MOD)
	lua_getfield(L, -1, "fmod");
	lua_setfield(L, -2, "mod");
#endif
	return 1;
}


//-------------------------------------------------------------linit.c--------------------------------------------------------

//TODO add these libs as needed

static const luaL_Reg lualibs[] = {
	//{"", luaopen_base},
	//{LUA_LOADLIBNAME, luaopen_package},
	//{LUA_TABLIBNAME, luaopen_table},
	//{LUA_IOLIBNAME, luaopen_io},
	//{LUA_OSLIBNAME, luaopen_os},
	//{LUA_STRLIBNAME, luaopen_string},
	{LUA_MATHLIBNAME, luaopen_math},
	//{LUA_DBLIBNAME, luaopen_debug},
	{NULL, NULL}
};


LUALIB_API void luaL_openlibs (lua_State *L) {
	const luaL_Reg *lib = lualibs;
	for (; lib->func; lib++) {
		lua_pushcfunction(L, lib->func);
		lua_pushstring(L, lib->name);
		lua_call(L, 1, 0);
	}
}



#endif
