#pragma once

#include <stdbool.h>

/**
 * @brief Goto label if expression evaluates to false
 * 
 * @param ifExpr expression to check
 * @param label goto label
 */
#define IF_FAILED_GOTO( ifExpr, label )\
    do\
    {\
        if( !(ifExpr) )\
        {\
             goto label;\
        }\
    }while( false )

/**
 * @brief Goto label if pointer is equal to NULL
 * 
 * @param ptr pointer to check
 * @param label goto label
 */
#define IF_NULL_GOTO( ptr, label )\
    IF_FAILED_GOTO( (ptr) != NULL, label )

/**
 * @brief Set status variable and goto label if expression evaluates to false
 * 
 * @param ifExpr expression to check
 * @param statusVar statusvarible to set
 * @param statusValue value to set to status variable
 * @param label goto label
 */
#define IF_FAILED_SETSTATUS_GOTO( ifExpr, statusVar, statusValue, label )\
    do\
    {\
        if( !(ifExpr) )\
        {\
             (statusVar) = (statusValue);\
             goto label;\
        }\
    }while( false )

/**
 * @brief Set status variable and goto label if pointer is equal to NULL
 * 
 * @param ptr pointer to check
 * @param statusVar statusvarible to set
 * @param statusValue value to set to status variable
 * @param label goto label
 */
#define IF_NULL_SETSTATUS_GOTO( ptr, statusVar, statusValue, label )\
    IF_FAILED_SETSTATUS_GOTO( (ptr) != NULL, (statusVar), (statusValue), label )

/**
 * @brief Return from funtion if expression evaluates to false
 * 
 * @param ifExpr expression to check
 */
#define IF_FAILED_RETURN( ifExpr )\
    do\
    {\
        if( !(ifExpr) )\
        {\
             return;\
        }\
    }while( false )

/**
 * @brief Return from function if pointer equal to NULL
 * 
 * @param ptr pointer to check
 */
#define IF_NULL_RETURN( ptr )\
    IF_FAILED_RETURN( (ptr) != NULL )

/**
 * @brief Return an expression from function if expression evaluates to false
 * 
 * @param ifExpr expression to check
 * @param returnExpr expression to return
 */
#define IF_FAILED_RETURN_EXPR( ifExpr, returnExpr )\
    do\
    {\
        if( !(ifExpr) )\
        {\
             return (returnExpr);\
        }\
    }while( false )

/**
 * @brief Return an expression from function if point is equal to NULL
 * 
 * @param ptr pointer to check
 * @param returnExpr expression to return
 */
#define IF_NULL_RETURN_EXPR( ptr, returnExpr )\
    IF_FAILED_RETURN_EXPR( ptr != NULL, (returnExpr) )
