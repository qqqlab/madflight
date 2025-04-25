#include "src/lua.hpp" //include as C
#include <setjmp.h>
#include <Arduino.h>
#include "../hal/hal.h" //xTaskDelay

static jmp_buf luawrap_panic_jump;

bool luawrap_overtime = false;
uint32_t luawrap_start_ts = 0;

//debug handler, called every time 1000 lines are executed
static void luawrap_hook(lua_State *L, lua_Debug *ar) {
    return;
    
    Serial.printf("luawrap_hookcnt dt=%u\n", micros() - luawrap_start_ts);
    if(micros() - luawrap_start_ts < 1000000) return;

    luawrap_overtime = true;

    // we need to aggressively bail out as we are over time
    // so we will aggressively trap errors until we clear out
    lua_sethook(L, luawrap_hook, LUA_MASKCOUNT, 1);

    luaL_error(L, "Exceeded CPU time");
}

static void luawrap_reset_loop_overtime(lua_State *L) {
    luawrap_overtime = false;
    // reset the hook to clear the counter
    lua_sethook(L, luawrap_hook, LUA_MASKCOUNT, 1000);
    luawrap_start_ts = micros();
    Serial.println("luawrap_reset_loop_overtime");
}

/* custom panic handler */
static int luawrap_panic(lua_State *L)
{
  longjmp(luawrap_panic_jump, 1);
  /* will never return */
  return 0;
}

//lua print()
static int luawrap_print(lua_State *L) {
  int n = lua_gettop(L);  /* number of arguments */
  int i;
  lua_getglobal(L, "tostring");
  for (i=1; i<=n; i++) {
    const char *s;
    size_t l;
    lua_pushvalue(L, -1);  /* function to be called */
    lua_pushvalue(L, i);   /* value to print */
    lua_call(L, 1, 1);
    s = lua_tolstring(L, -1, &l);  /* get result */
    if (s == NULL)
      return luaL_error(L, "'tostring' must return a string to 'print'");
    if (i>1) Serial.write("\t");
    Serial.write(s);
    lua_pop(L, 1);  /* pop result */
  }
  Serial.println();
  return 0;
}

static int luawrap_millis(lua_State *L) {
  lua_pushinteger(L, millis());
  return 1;
}

static int luawrap_micros(lua_State *L) {
  lua_pushinteger(L, micros());
  return 1;
}  

static int luawrap_delay(lua_State *L) {
  int a = luaL_checkinteger(L, 1) / portTICK_PERIOD_MS;
  vTaskDelay(a);
  return 0;
}  

static void luawrap_dumpstack (lua_State *L) {
  int top = lua_gettop(L);
  for (int i = 1; i <= top; i++) {
    Serial.printf("%2d :%3d : %-7s: ", i, i - top - 1, luaL_typename(L,i));
    switch (lua_type(L, i)) {
      case LUA_TNUMBER:
        Serial.printf("%g\n", lua_tonumber(L,i));
        break;
      case LUA_TSTRING:
        Serial.printf("\"%s\"\n", lua_tostring(L,i));
        break;
      case LUA_TBOOLEAN:
        Serial.printf("%s\n", (lua_toboolean(L, i) ? "true" : "false"));
        break;
      case LUA_TNIL:
        Serial.printf("%s\n", "nil");
        break;
      default:
        Serial.printf("%p\n", lua_topointer(L,i));
        break;
    }
  }
}

static lua_State *L = NULL;

void luawrap_run(const char* code) {
  L = luaL_newstate();

  lua_atpanic(L, luawrap_panic); //first thing to do
  if (setjmp(luawrap_panic_jump) != 0) {
    // execution will resume here if a lua_xxx() call generates a panic which would otherwise hang the application
    const char *err = luaL_checkstring(L, -1);
    Serial.printf("LUA: PANIC %s\n",err);      
    lua_close(L);
    return;
  }

  luaL_openlibs(L);

  lua_register(L, "print", luawrap_print); //c function for lua print()

  //register mf table
  lua_createtable(L, 0, 2);
  lua_pushcfunction(L, luawrap_millis);
  lua_setfield(L, -2, "millis");
  lua_pushcfunction(L, luawrap_micros);
  lua_setfield(L, -2, "micros");
  lua_pushcfunction(L, luawrap_delay);
  lua_setfield(L, -2, "delay");
  lua_setglobal(L, "mf");

  int result = 0;
  uint32_t dt = micros();
  //execute
  //luawrap_reset_loop_overtime(L); //optional
  if (luaL_dostring(L, code) == LUA_OK) {
    lua_pop(L, lua_gettop(L)); // Pop the return value
  }else{
    const char *err = luaL_checkstring(L, -1);
    Serial.printf("LUA: ERROR: %s\n", err);
  }
  lua_close(L);
}