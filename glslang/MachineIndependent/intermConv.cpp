//
// Copyright (C) 2002-2005  3Dlabs Inc. Ltd.
// Copyright (C) 2012-2016 LunarG, Inc.
// Copyright (C) 2017 ARM Limited.
// Modifications Copyright (C) 2020 Advanced Micro Devices, Inc. All rights reserved.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
//    Neither the name of 3Dlabs Inc. Ltd. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#if !defined(GLSLANG_WEB) && !defined(GLSLANG_ANGLE)

#include "localintermediate.h"
#include "../Include/InfoSink.h"

#ifdef _MSC_VER
#include <cfloat>
#else
#include <cmath>
#endif
#include <cstdint>
#include <stack>
#include <unordered_set>

namespace {

bool IsInfinity(double x) {
#ifdef _MSC_VER
    switch (_fpclass(x)) {
    case _FPCLASS_NINF:
    case _FPCLASS_PINF:
        return true;
    default:
        return false;
    }
#else
    return std::isinf(x);
#endif
}

bool IsNan(double x) {
#ifdef _MSC_VER
    switch (_fpclass(x)) {
    case _FPCLASS_SNAN:
    case _FPCLASS_QNAN:
        return true;
    default:
        return false;
    }
#else
  return std::isnan(x);
#endif
}

}

namespace glslang {

//
// Two purposes:
// 1.  Show an example of how to iterate tree.  Functions can
//     also directly call Traverse() on children themselves to
//     have finer grained control over the process than shown here.
//     See the last function for how to get started.
// 2.  Print out a text based description of the tree.
//

//
// Use this class to carry along data from node to node in
// the traversal
//
class TConvertTraverser : public TIntermTraverser {
public:
    TConvertTraverser(TInfoSink& i) : TIntermTraverser(true, true, true, false),
        infoSink(i), extraOutput(NoExtraOutput), lastLine(0), lastFile(0), level(0),
        declaringSymbol(0), skipped(false), swizzling(false) {
        sequenceSeperator.push("");
        sequenceEnd.push("");
    }

    enum EExtraOutput {
        NoExtraOutput,
        BinaryDoubleOutput
    };
    void setDoubleOutput(EExtraOutput extra) { extraOutput = extra; }

    virtual bool visitBinary(TVisit, TIntermBinary* node);
    virtual bool visitUnary(TVisit, TIntermUnary* node);
    virtual bool visitAggregate(TVisit, TIntermAggregate* node);
    virtual bool visitSelection(TVisit, TIntermSelection* node);
    virtual void visitConstantUnion(TIntermConstantUnion* node);
    virtual void visitSymbol(TIntermSymbol* node);
    virtual bool visitLoop(TVisit, TIntermLoop* node);
    virtual bool visitBranch(TVisit, TIntermBranch* node);
    virtual bool visitSwitch(TVisit, TIntermSwitch* node);

    TInfoSink& infoSink;
    int lastLine;
    int lastFile;
    int level;
    int declaringSymbol;
    bool skipped;
    bool swizzling;
    std::stack<const char*> sequenceSeperator;
    std::stack<const char*> sequenceEnd;
    std::unordered_set<TString> globalSymbols;
    std::unordered_set<TString> localSymbols;
protected:
    TConvertTraverser(TConvertTraverser&);
    TConvertTraverser& operator=(TConvertTraverser&);

    EExtraOutput extraOutput;
private:
    void tryNewLine(const TIntermNode* node) {
        if (lastFile != node->getLoc().string || node->getLoc().line == 0 || lastLine < node->getLoc().line) {
            infoSink.debug << "\n";
            if (node->getLoc().line != 0 && (lastFile != node->getLoc().string || lastLine + 1 != node->getLoc().line)) {
                infoSink.debug << "#line " << node->getLoc().line << " " << node->getLoc().string << "\n";
            }
            for (int i = 0; i < level; ++i) {
                infoSink.debug << "    ";
            }
            lastLine = node->getLoc().line;
            lastFile = node->getLoc().string;
        }
    }
    void gotoNewLine() {
        infoSink.debug << "\n";
        for (int i = 0; i < level; ++i) {
            infoSink.debug << "    ";
        }
    }
};

//
// The rest of the file are the traversal functions.  The last one
// is the one that starts the traversal.
//
// Return true from interior nodes to have the external traversal
// continue on to children.  If you process children yourself,
// return false.
//

bool TConvertTraverser::visitBinary(TVisit visit, TIntermBinary* node)
{
    TInfoSink& out = infoSink;

    if (visit == EvPreVisit) {
        tryNewLine(node);
        switch (node->getOp()) {
        case EOpAdd:
        case EOpSub:
        case EOpMul:
        case EOpDiv:
        case EOpMod:
        case EOpAnd:
        case EOpInclusiveOr:
        case EOpExclusiveOr:
        case EOpVectorTimesScalar:
        case EOpVectorTimesMatrix:
        case EOpMatrixTimesVector:
        case EOpMatrixTimesScalar:
        case EOpMatrixTimesMatrix:
        case EOpLogicalOr:
        case EOpLogicalXor:
        case EOpLogicalAnd: out.debug << "("; break;
        default: out.debug << "";
        }
    }
    else if (visit == EvInVisit) {
        switch (node->getOp()) {
        case EOpAssign:                   out.debug << " = ";   break;
        case EOpAddAssign:                out.debug << " += ";  break;
        case EOpSubAssign:                out.debug << " -= ";  break;
        case EOpMulAssign:                out.debug << " *= ";  break;
        case EOpVectorTimesMatrixAssign:  out.debug << " *= ";  break;
        case EOpVectorTimesScalarAssign:  out.debug << " *= ";  break;
        case EOpMatrixTimesScalarAssign:  out.debug << " *= ";  break;
        case EOpMatrixTimesMatrixAssign:  out.debug << " *= ";  break;
        case EOpDivAssign:                out.debug << " /= ";  break;
        case EOpModAssign:                out.debug << " %= ";  break;
        case EOpAndAssign:                out.debug << " &= ";  break;
        case EOpInclusiveOrAssign:        out.debug << " |= ";  break;
        case EOpExclusiveOrAssign:        out.debug << " ^= ";  break;
        case EOpLeftShiftAssign:          out.debug << " <<= "; break;
        case EOpRightShiftAssign:         out.debug << " >>= "; break;

        case EOpIndexDirect:
        case EOpIndexIndirect: out.debug << "["; break;
        case EOpIndexDirectStruct:
            {
                if (!skipped) {
                    out.debug << ".";
                }
                else {
                    skipped = false;
                }
                bool reference = node->getLeft()->getType().isReference();
                const TTypeList *members = reference ? node->getLeft()->getType().getReferentType()->getStruct() : node->getLeft()->getType().getStruct();
                out.debug << (*members)[node->getRight()->getAsConstantUnion()->getConstArray()[0].getIConst()].type->getFieldName();
                return false;
            }
        case EOpVectorSwizzle: {
            swizzling = true;
            sequenceSeperator.push("");
            sequenceEnd.push("");
            out.debug << ".";
            break;
        }
        case EOpMatrixSwizzle: out.debug << "matrix swizzle"; break;

        case EOpAdd:    out.debug << " + "; break;
        case EOpSub:    out.debug << " - "; break;
        case EOpMul:    out.debug << " * "; break;
        case EOpDiv:    out.debug << " / "; break;
        case EOpMod:    out.debug << " % "; break;
        case EOpRightShift:  out.debug << " >> "; break;
        case EOpLeftShift:   out.debug << " << "; break;
        case EOpAnd:         out.debug << " & ";  break;
        case EOpInclusiveOr: out.debug << " | ";  break;
        case EOpExclusiveOr: out.debug << " ^ ";  break;
        case EOpEqual:            out.debug << " == "; break;
        case EOpNotEqual:         out.debug << " != "; break;
        case EOpLessThan:         out.debug << " < ";  break;
        case EOpGreaterThan:      out.debug << " > ";  break;
        case EOpLessThanEqual:    out.debug << " <= "; break;
        case EOpGreaterThanEqual: out.debug << " >= "; break;
        case EOpVectorEqual:      out.debug << " == "; break;
        case EOpVectorNotEqual:   out.debug << " != "; break;

        case EOpVectorTimesScalar: out.debug << " * "; break;
        case EOpVectorTimesMatrix: out.debug << " * "; break;
        case EOpMatrixTimesVector: out.debug << " * "; break;
        case EOpMatrixTimesScalar: out.debug << " * "; break;
        case EOpMatrixTimesMatrix: out.debug << " * "; break;

        case EOpLogicalOr:  out.debug << " || "; break;
        case EOpLogicalXor: out.debug << " ^^ "; break;
        case EOpLogicalAnd: out.debug << " && "; break;

        case EOpAbsDifference:          out.debug << "absoluteDifference";    break;
        case EOpAddSaturate:            out.debug << "addSaturate";           break;
        case EOpSubSaturate:            out.debug << "subtractSaturate";      break;
        case EOpAverage:                out.debug << "average";               break;
        case EOpAverageRounded:         out.debug << "averageRounded";        break;
        case EOpMul32x16:               out.debug << "multiply32x16";         break;

        default: out.debug << "<unknown op>";
        }
    }
    else if (visit == EvPostVisit) {
        switch (node->getOp()) {
        case EOpIndexDirect:
        case EOpIndexIndirect:   out.debug << "]";   break;
        case EOpVectorSwizzle: {
            swizzling = false;
            sequenceSeperator.pop();
            sequenceEnd.pop();
            break;
        }
        case EOpAdd:
        case EOpSub:
        case EOpMul:
        case EOpDiv:
        case EOpMod:
        case EOpAnd:
        case EOpInclusiveOr:
        case EOpExclusiveOr:
        case EOpVectorTimesScalar:
        case EOpVectorTimesMatrix:
        case EOpMatrixTimesVector:
        case EOpMatrixTimesScalar:
        case EOpMatrixTimesMatrix:
        case EOpLogicalOr:
        case EOpLogicalXor:
        case EOpLogicalAnd: out.debug << ")"; break;
        default: out.debug << "";
        }
    }

    return true;
}

bool TConvertTraverser::visitUnary(TVisit visit, TIntermUnary* node)
{
    TInfoSink& out = infoSink;

    if (visit == EvPreVisit) {
        tryNewLine(node);
        switch (node->getOp()) {
        case EOpNegative:       out.debug << "-";         break;
        case EOpVectorLogicalNot: out.debug << "not(";   break;
        case EOpLogicalNot:     out.debug << "!";   break;
        case EOpBitwiseNot:     out.debug << "~";          break;

        case EOpPreIncrement:   out.debug << "++";        break;
        case EOpPreDecrement:   out.debug << "--";        break;
        case EOpCopyObject:     out.debug << "copy object";          break;

        // * -> bool
        case EOpConvInt8ToBool:
        case EOpConvUint8ToBool:
        case EOpConvInt16ToBool:
        case EOpConvUint16ToBool:
        case EOpConvIntToBool:
        case EOpConvUintToBool:
        case EOpConvInt64ToBool:
        case EOpConvUint64ToBool:
        case EOpConvFloat16ToBool:
        case EOpConvFloatToBool:
        case EOpConvDoubleToBool:

        // bool -> *
        case EOpConvBoolToInt8:
        case EOpConvBoolToUint8:
        case EOpConvBoolToInt16:
        case EOpConvBoolToUint16:
        case EOpConvBoolToInt:
        case EOpConvBoolToUint:
        case EOpConvBoolToInt64:
        case EOpConvBoolToUint64:
        case EOpConvBoolToFloat16:
        case EOpConvBoolToFloat:
        case EOpConvBoolToDouble:

        // int8_t -> (u)int*
        case EOpConvInt8ToInt16:
        case EOpConvInt8ToInt:
        case EOpConvInt8ToInt64:
        case EOpConvInt8ToUint8:
        case EOpConvInt8ToUint16:
        case EOpConvInt8ToUint:
        case EOpConvInt8ToUint64:

        // uint8_t -> (u)int*
        case EOpConvUint8ToInt8:
        case EOpConvUint8ToInt16:
        case EOpConvUint8ToInt:
        case EOpConvUint8ToInt64:
        case EOpConvUint8ToUint16:
        case EOpConvUint8ToUint:
        case EOpConvUint8ToUint64:

        // int8_t -> float*
        case EOpConvInt8ToFloat16:
        case EOpConvInt8ToFloat:
        case EOpConvInt8ToDouble:

        // uint8_t -> float*
        case EOpConvUint8ToFloat16:
        case EOpConvUint8ToFloat:
        case EOpConvUint8ToDouble:

        // int16_t -> (u)int*
        case EOpConvInt16ToInt8:
        case EOpConvInt16ToInt:
        case EOpConvInt16ToInt64:
        case EOpConvInt16ToUint8:
        case EOpConvInt16ToUint16:
        case EOpConvInt16ToUint:
        case EOpConvInt16ToUint64:

        // int16_t -> float*
        case EOpConvInt16ToFloat16:
        case EOpConvInt16ToFloat:
        case EOpConvInt16ToDouble:

        // uint16_t -> (u)int*
        case EOpConvUint16ToInt8:
        case EOpConvUint16ToInt16:
        case EOpConvUint16ToInt:
        case EOpConvUint16ToInt64:
        case EOpConvUint16ToUint8:
        case EOpConvUint16ToUint:
        case EOpConvUint16ToUint64:

        // uint16_t -> float*
        case EOpConvUint16ToFloat16:
        case EOpConvUint16ToFloat:
        case EOpConvUint16ToDouble:

        // int32_t -> (u)int*
        case EOpConvIntToInt8:
        case EOpConvIntToInt16:
        case EOpConvIntToInt64:
        case EOpConvIntToUint8:
        case EOpConvIntToUint16:
        case EOpConvIntToUint:
        case EOpConvIntToUint64:

        // int32_t -> float*
        case EOpConvIntToFloat16:
        case EOpConvIntToFloat:
        case EOpConvIntToDouble:

        // uint32_t -> (u)int*
        case EOpConvUintToInt8:
        case EOpConvUintToInt16:
        case EOpConvUintToInt:
        case EOpConvUintToInt64:
        case EOpConvUintToUint8:
        case EOpConvUintToUint16:
        case EOpConvUintToUint64:

        // uint32_t -> float*
        case EOpConvUintToFloat16:
        case EOpConvUintToFloat:
        case EOpConvUintToDouble:

        // int64 -> (u)int*
        case EOpConvInt64ToInt8:
        case EOpConvInt64ToInt16:
        case EOpConvInt64ToInt:
        case EOpConvInt64ToUint8:
        case EOpConvInt64ToUint16:
        case EOpConvInt64ToUint:
        case EOpConvInt64ToUint64:

         // int64 -> float*
        case EOpConvInt64ToFloat16:
        case EOpConvInt64ToFloat:
        case EOpConvInt64ToDouble:

        // uint64 -> (u)int*
        case EOpConvUint64ToInt8:
        case EOpConvUint64ToInt16:
        case EOpConvUint64ToInt:
        case EOpConvUint64ToInt64:
        case EOpConvUint64ToUint8:
        case EOpConvUint64ToUint16:
        case EOpConvUint64ToUint:

        // uint64 -> float*
        case EOpConvUint64ToFloat16:
        case EOpConvUint64ToFloat:
        case EOpConvUint64ToDouble:

        // float16_t -> int*
        case EOpConvFloat16ToInt8:
        case EOpConvFloat16ToInt16:
        case EOpConvFloat16ToInt:
        case EOpConvFloat16ToInt64:

        // float16_t -> uint*
        case EOpConvFloat16ToUint8:
        case EOpConvFloat16ToUint16:
        case EOpConvFloat16ToUint:
        case EOpConvFloat16ToUint64:

        // float16_t -> float*
        case EOpConvFloat16ToFloat:
        case EOpConvFloat16ToDouble:

        // float32 -> float*
        case EOpConvFloatToFloat16:
        case EOpConvFloatToDouble:

        // float32_t -> int*
        case EOpConvFloatToInt8:
        case EOpConvFloatToInt16:
        case EOpConvFloatToInt:
        case EOpConvFloatToInt64:

        // float32_t -> uint*
        case EOpConvFloatToUint8:
        case EOpConvFloatToUint16:
        case EOpConvFloatToUint:
        case EOpConvFloatToUint64:

        // double -> float*
        case EOpConvDoubleToFloat16:
        case EOpConvDoubleToFloat:

        // double -> int*
        case EOpConvDoubleToInt8:
        case EOpConvDoubleToInt16:
        case EOpConvDoubleToInt:
        case EOpConvDoubleToInt64:

        // float32_t -> uint*
        case EOpConvDoubleToUint8:
        case EOpConvDoubleToUint16:
        case EOpConvDoubleToUint:
        case EOpConvDoubleToUint64: out.debug << node->getTypeString() << "("; break;

        case EOpConvUint64ToPtr:  out.debug << "Convert uint64_t to pointer";   break;
        case EOpConvPtrToUint64:  out.debug << "Convert pointer to uint64_t";   break;

        case EOpConvUint64ToAccStruct: out.debug << "Convert uint64_t to acceleration structure"; break;
        case EOpConvUvec2ToAccStruct:  out.debug << "Convert uvec2 to acceleration strucuture "; break;

        case EOpRadians:        out.debug << "radians("; break;
        case EOpDegrees:        out.debug << "degrees("; break;
        case EOpSin:            out.debug << "sin(";     break;
        case EOpCos:            out.debug << "cos(";     break;
        case EOpTan:            out.debug << "tan(";     break;
        case EOpAsin:           out.debug << "asin(";    break;
        case EOpAcos:           out.debug << "acos(";    break;
        case EOpAtan:           out.debug << "atan(";    break;
        case EOpSinh:           out.debug << "sinh(";    break;
        case EOpCosh:           out.debug << "cosh(";    break;
        case EOpTanh:           out.debug << "tanh(";    break;
        case EOpAsinh:          out.debug << "asinh(";   break;
        case EOpAcosh:          out.debug << "acosh(";   break;
        case EOpAtanh:          out.debug << "atanh(";   break;

        case EOpExp:            out.debug << "exp(";         break;
        case EOpLog:            out.debug << "log(";         break;
        case EOpExp2:           out.debug << "exp2(";        break;
        case EOpLog2:           out.debug << "log2(";        break;
        case EOpSqrt:           out.debug << "sqrt(";        break;
        case EOpInverseSqrt:    out.debug << "inversesqrt("; break;

        case EOpAbs:            out.debug << "abs(";       break;
        case EOpSign:           out.debug << "sign(";                 break;
        case EOpFloor:          out.debug << "floor(";                break;
        case EOpTrunc:          out.debug << "trunc(";                break;
        case EOpRound:          out.debug << "round(";                break;
        case EOpRoundEven:      out.debug << "roundEven(";            break;
        case EOpCeil:           out.debug << "ceil(";              break;
        case EOpFract:          out.debug << "fract(";             break;

        case EOpIsNan:          out.debug << "isnan(";                break;
        case EOpIsInf:          out.debug << "isinf(";                break;

        case EOpFloatBitsToInt: out.debug << "floatBitsToInt(";       break;
        case EOpFloatBitsToUint:out.debug << "floatBitsToUint(";      break;
        case EOpIntBitsToFloat: out.debug << "intBitsToFloat(";       break;
        case EOpUintBitsToFloat:out.debug << "uintBitsToFloat(";      break;
        case EOpDoubleBitsToInt64:  out.debug << "doubleBitsToInt64";  break;
        case EOpDoubleBitsToUint64: out.debug << "doubleBitsToUint64"; break;
        case EOpInt64BitsToDouble:  out.debug << "int64BitsToDouble";  break;
        case EOpUint64BitsToDouble: out.debug << "uint64BitsToDouble"; break;
        case EOpFloat16BitsToInt16:  out.debug << "float16BitsToInt16";  break;
        case EOpFloat16BitsToUint16: out.debug << "float16BitsToUint16"; break;
        case EOpInt16BitsToFloat16:  out.debug << "int16BitsToFloat16";  break;
        case EOpUint16BitsToFloat16: out.debug << "uint16BitsToFloat16"; break;

        case EOpPackSnorm2x16:  out.debug << "packSnorm2x16(";        break;
        case EOpUnpackSnorm2x16:out.debug << "unpackSnorm2x16(";      break;
        case EOpPackUnorm2x16:  out.debug << "packUnorm2x16(";        break;
        case EOpUnpackUnorm2x16:out.debug << "unpackUnorm2x16(";      break;
        case EOpPackHalf2x16:   out.debug << "packHalf2x16(";         break;
        case EOpUnpackHalf2x16: out.debug << "unpackHalf2x16(";       break;
        case EOpPack16:           out.debug << "pack16";                 break;
        case EOpPack32:           out.debug << "pack32";                 break;
        case EOpPack64:           out.debug << "pack64";                 break;
        case EOpUnpack32:         out.debug << "unpack32";               break;
        case EOpUnpack16:         out.debug << "unpack16";               break;
        case EOpUnpack8:          out.debug << "unpack8";               break;

        case EOpPackSnorm4x8:     out.debug << "PackSnorm4x8";       break;
        case EOpUnpackSnorm4x8:   out.debug << "UnpackSnorm4x8";     break;
        case EOpPackUnorm4x8:     out.debug << "PackUnorm4x8";       break;
        case EOpUnpackUnorm4x8:   out.debug << "UnpackUnorm4x8";     break;
        case EOpPackDouble2x32:   out.debug << "PackDouble2x32";     break;
        case EOpUnpackDouble2x32: out.debug << "UnpackDouble2x32";   break;

        case EOpPackInt2x32:      out.debug << "packInt2x32";        break;
        case EOpUnpackInt2x32:    out.debug << "unpackInt2x32";      break;
        case EOpPackUint2x32:     out.debug << "packUint2x32";       break;
        case EOpUnpackUint2x32:   out.debug << "unpackUint2x32";     break;

        case EOpPackInt2x16:      out.debug << "packInt2x16";        break;
        case EOpUnpackInt2x16:    out.debug << "unpackInt2x16";      break;
        case EOpPackUint2x16:     out.debug << "packUint2x16";       break;
        case EOpUnpackUint2x16:   out.debug << "unpackUint2x16";     break;

        case EOpPackInt4x16:      out.debug << "packInt4x16";        break;
        case EOpUnpackInt4x16:    out.debug << "unpackInt4x16";      break;
        case EOpPackUint4x16:     out.debug << "packUint4x16";       break;
        case EOpUnpackUint4x16:   out.debug << "unpackUint4x16";     break;
        case EOpPackFloat2x16:    out.debug << "packFloat2x16";      break;
        case EOpUnpackFloat2x16:  out.debug << "unpackFloat2x16";    break;

        case EOpLength:         out.debug << "length(";               break;
        case EOpNormalize:      out.debug << "normalize(";            break;
        case EOpDPdx:           out.debug << "dPdx";                 break;
        case EOpDPdy:           out.debug << "dPdy";                 break;
        case EOpFwidth:         out.debug << "fwidth(";               break;
        case EOpDPdxFine:       out.debug << "dPdxFine";             break;
        case EOpDPdyFine:       out.debug << "dPdyFine";             break;
        case EOpFwidthFine:     out.debug << "fwidthFine";           break;
        case EOpDPdxCoarse:     out.debug << "dPdxCoarse";           break;
        case EOpDPdyCoarse:     out.debug << "dPdyCoarse";           break;
        case EOpFwidthCoarse:   out.debug << "fwidthCoarse";         break;

        case EOpInterpolateAtCentroid: out.debug << "interpolateAtCentroid";  break;

        case EOpDeterminant:    out.debug << "determinant(";          break;
        case EOpMatrixInverse:  out.debug << "inverse(";              break;
        case EOpTranspose:      out.debug << "transpose(";            break;

        case EOpAny:            out.debug << "any(";                  break;
        case EOpAll:            out.debug << "all(";                  break;

        case EOpArrayLength:    out.debug << "array length";         break;

        case EOpEmitStreamVertex:   out.debug << "EmitStreamVertex";   break;
        case EOpEndStreamPrimitive: out.debug << "EndStreamPrimitive"; break;

        case EOpAtomicCounterIncrement: out.debug << "AtomicCounterIncrement";break;
        case EOpAtomicCounterDecrement: out.debug << "AtomicCounterDecrement";break;
        case EOpAtomicCounter:          out.debug << "AtomicCounter";         break;

        case EOpTextureQuerySize:       out.debug << "textureSize";           break;
        case EOpTextureQueryLod:        out.debug << "textureQueryLod";       break;
        case EOpTextureQueryLevels:     out.debug << "textureQueryLevels";    break;
        case EOpTextureQuerySamples:    out.debug << "textureSamples";        break;
        case EOpImageQuerySize:         out.debug << "imageQuerySize";        break;
        case EOpImageQuerySamples:      out.debug << "imageQuerySamples";     break;
        case EOpImageLoad:              out.debug << "imageLoad";             break;

        case EOpBitFieldReverse:        out.debug << "bitFieldReverse";       break;
        case EOpBitCount:               out.debug << "bitCount";              break;
        case EOpFindLSB:                out.debug << "findLSB";               break;
        case EOpFindMSB:                out.debug << "findMSB";               break;

        case EOpCountLeadingZeros:      out.debug << "countLeadingZeros";     break;
        case EOpCountTrailingZeros:     out.debug << "countTrailingZeros";    break;

        case EOpNoise:                  out.debug << "noise";                 break;

        case EOpBallot:                 out.debug << "ballot";                break;
        case EOpReadFirstInvocation:    out.debug << "readFirstInvocation";   break;

        case EOpAnyInvocation:          out.debug << "anyInvocation";         break;
        case EOpAllInvocations:         out.debug << "allInvocations";        break;
        case EOpAllInvocationsEqual:    out.debug << "allInvocationsEqual";   break;

        case EOpSubgroupElect:                   out.debug << "subgroupElect";                   break;
        case EOpSubgroupAll:                     out.debug << "subgroupAll";                     break;
        case EOpSubgroupAny:                     out.debug << "subgroupAny";                     break;
        case EOpSubgroupAllEqual:                out.debug << "subgroupAllEqual";                break;
        case EOpSubgroupBroadcast:               out.debug << "subgroupBroadcast";               break;
        case EOpSubgroupBroadcastFirst:          out.debug << "subgroupBroadcastFirst";          break;
        case EOpSubgroupBallot:                  out.debug << "subgroupBallot";                  break;
        case EOpSubgroupInverseBallot:           out.debug << "subgroupInverseBallot";           break;
        case EOpSubgroupBallotBitExtract:        out.debug << "subgroupBallotBitExtract";        break;
        case EOpSubgroupBallotBitCount:          out.debug << "subgroupBallotBitCount";          break;
        case EOpSubgroupBallotInclusiveBitCount: out.debug << "subgroupBallotInclusiveBitCount"; break;
        case EOpSubgroupBallotExclusiveBitCount: out.debug << "subgroupBallotExclusiveBitCount"; break;
        case EOpSubgroupBallotFindLSB:           out.debug << "subgroupBallotFindLSB";           break;
        case EOpSubgroupBallotFindMSB:           out.debug << "subgroupBallotFindMSB";           break;
        case EOpSubgroupShuffle:                 out.debug << "subgroupShuffle";                 break;
        case EOpSubgroupShuffleXor:              out.debug << "subgroupShuffleXor";              break;
        case EOpSubgroupShuffleUp:               out.debug << "subgroupShuffleUp";               break;
        case EOpSubgroupShuffleDown:             out.debug << "subgroupShuffleDown";             break;
        case EOpSubgroupAdd:                     out.debug << "subgroupAdd";                     break;
        case EOpSubgroupMul:                     out.debug << "subgroupMul";                     break;
        case EOpSubgroupMin:                     out.debug << "subgroupMin";                     break;
        case EOpSubgroupMax:                     out.debug << "subgroupMax";                     break;
        case EOpSubgroupAnd:                     out.debug << "subgroupAnd";                     break;
        case EOpSubgroupOr:                      out.debug << "subgroupOr";                      break;
        case EOpSubgroupXor:                     out.debug << "subgroupXor";                     break;
        case EOpSubgroupInclusiveAdd:            out.debug << "subgroupInclusiveAdd";            break;
        case EOpSubgroupInclusiveMul:            out.debug << "subgroupInclusiveMul";            break;
        case EOpSubgroupInclusiveMin:            out.debug << "subgroupInclusiveMin";            break;
        case EOpSubgroupInclusiveMax:            out.debug << "subgroupInclusiveMax";            break;
        case EOpSubgroupInclusiveAnd:            out.debug << "subgroupInclusiveAnd";            break;
        case EOpSubgroupInclusiveOr:             out.debug << "subgroupInclusiveOr";             break;
        case EOpSubgroupInclusiveXor:            out.debug << "subgroupInclusiveXor";            break;
        case EOpSubgroupExclusiveAdd:            out.debug << "subgroupExclusiveAdd";            break;
        case EOpSubgroupExclusiveMul:            out.debug << "subgroupExclusiveMul";            break;
        case EOpSubgroupExclusiveMin:            out.debug << "subgroupExclusiveMin";            break;
        case EOpSubgroupExclusiveMax:            out.debug << "subgroupExclusiveMax";            break;
        case EOpSubgroupExclusiveAnd:            out.debug << "subgroupExclusiveAnd";            break;
        case EOpSubgroupExclusiveOr:             out.debug << "subgroupExclusiveOr";             break;
        case EOpSubgroupExclusiveXor:            out.debug << "subgroupExclusiveXor";            break;
        case EOpSubgroupClusteredAdd:            out.debug << "subgroupClusteredAdd";            break;
        case EOpSubgroupClusteredMul:            out.debug << "subgroupClusteredMul";            break;
        case EOpSubgroupClusteredMin:            out.debug << "subgroupClusteredMin";            break;
        case EOpSubgroupClusteredMax:            out.debug << "subgroupClusteredMax";            break;
        case EOpSubgroupClusteredAnd:            out.debug << "subgroupClusteredAnd";            break;
        case EOpSubgroupClusteredOr:             out.debug << "subgroupClusteredOr";             break;
        case EOpSubgroupClusteredXor:            out.debug << "subgroupClusteredXor";            break;
        case EOpSubgroupQuadBroadcast:           out.debug << "subgroupQuadBroadcast";           break;
        case EOpSubgroupQuadSwapHorizontal:      out.debug << "subgroupQuadSwapHorizontal";      break;
        case EOpSubgroupQuadSwapVertical:        out.debug << "subgroupQuadSwapVertical";        break;
        case EOpSubgroupQuadSwapDiagonal:        out.debug << "subgroupQuadSwapDiagonal";        break;

        case EOpSubgroupPartition:                          out.debug << "subgroupPartitionNV";                          break;
        case EOpSubgroupPartitionedAdd:                     out.debug << "subgroupPartitionedAddNV";                     break;
        case EOpSubgroupPartitionedMul:                     out.debug << "subgroupPartitionedMulNV";                     break;
        case EOpSubgroupPartitionedMin:                     out.debug << "subgroupPartitionedMinNV";                     break;
        case EOpSubgroupPartitionedMax:                     out.debug << "subgroupPartitionedMaxNV";                     break;
        case EOpSubgroupPartitionedAnd:                     out.debug << "subgroupPartitionedAndNV";                     break;
        case EOpSubgroupPartitionedOr:                      out.debug << "subgroupPartitionedOrNV";                      break;
        case EOpSubgroupPartitionedXor:                     out.debug << "subgroupPartitionedXorNV";                     break;
        case EOpSubgroupPartitionedInclusiveAdd:            out.debug << "subgroupPartitionedInclusiveAddNV";            break;
        case EOpSubgroupPartitionedInclusiveMul:            out.debug << "subgroupPartitionedInclusiveMulNV";            break;
        case EOpSubgroupPartitionedInclusiveMin:            out.debug << "subgroupPartitionedInclusiveMinNV";            break;
        case EOpSubgroupPartitionedInclusiveMax:            out.debug << "subgroupPartitionedInclusiveMaxNV";            break;
        case EOpSubgroupPartitionedInclusiveAnd:            out.debug << "subgroupPartitionedInclusiveAndNV";            break;
        case EOpSubgroupPartitionedInclusiveOr:             out.debug << "subgroupPartitionedInclusiveOrNV";             break;
        case EOpSubgroupPartitionedInclusiveXor:            out.debug << "subgroupPartitionedInclusiveXorNV";            break;
        case EOpSubgroupPartitionedExclusiveAdd:            out.debug << "subgroupPartitionedExclusiveAddNV";            break;
        case EOpSubgroupPartitionedExclusiveMul:            out.debug << "subgroupPartitionedExclusiveMulNV";            break;
        case EOpSubgroupPartitionedExclusiveMin:            out.debug << "subgroupPartitionedExclusiveMinNV";            break;
        case EOpSubgroupPartitionedExclusiveMax:            out.debug << "subgroupPartitionedExclusiveMaxNV";            break;
        case EOpSubgroupPartitionedExclusiveAnd:            out.debug << "subgroupPartitionedExclusiveAndNV";            break;
        case EOpSubgroupPartitionedExclusiveOr:             out.debug << "subgroupPartitionedExclusiveOrNV";             break;
        case EOpSubgroupPartitionedExclusiveXor:            out.debug << "subgroupPartitionedExclusiveXorNV";            break;

        case EOpClip:                   out.debug << "clip";                  break;
        case EOpIsFinite:               out.debug << "isfinite";              break;
        case EOpLog10:                  out.debug << "log10";                 break;
        case EOpRcp:                    out.debug << "rcp";                   break;
        case EOpSaturate:               out.debug << "saturate";              break;

        case EOpSparseTexelsResident:   out.debug << "sparseTexelsResident";  break;

        case EOpMinInvocations:             out.debug << "minInvocations";              break;
        case EOpMaxInvocations:             out.debug << "maxInvocations";              break;
        case EOpAddInvocations:             out.debug << "addInvocations";              break;
        case EOpMinInvocationsNonUniform:   out.debug << "minInvocationsNonUniform";    break;
        case EOpMaxInvocationsNonUniform:   out.debug << "maxInvocationsNonUniform";    break;
        case EOpAddInvocationsNonUniform:   out.debug << "addInvocationsNonUniform";    break;

        case EOpMinInvocationsInclusiveScan:            out.debug << "minInvocationsInclusiveScan";             break;
        case EOpMaxInvocationsInclusiveScan:            out.debug << "maxInvocationsInclusiveScan";             break;
        case EOpAddInvocationsInclusiveScan:            out.debug << "addInvocationsInclusiveScan";             break;
        case EOpMinInvocationsInclusiveScanNonUniform:  out.debug << "minInvocationsInclusiveScanNonUniform";   break;
        case EOpMaxInvocationsInclusiveScanNonUniform:  out.debug << "maxInvocationsInclusiveScanNonUniform";   break;
        case EOpAddInvocationsInclusiveScanNonUniform:  out.debug << "addInvocationsInclusiveScanNonUniform";   break;

        case EOpMinInvocationsExclusiveScan:            out.debug << "minInvocationsExclusiveScan";             break;
        case EOpMaxInvocationsExclusiveScan:            out.debug << "maxInvocationsExclusiveScan";             break;
        case EOpAddInvocationsExclusiveScan:            out.debug << "addInvocationsExclusiveScan";             break;
        case EOpMinInvocationsExclusiveScanNonUniform:  out.debug << "minInvocationsExclusiveScanNonUniform";   break;
        case EOpMaxInvocationsExclusiveScanNonUniform:  out.debug << "maxInvocationsExclusiveScanNonUniform";   break;
        case EOpAddInvocationsExclusiveScanNonUniform:  out.debug << "addInvocationsExclusiveScanNonUniform";   break;

        case EOpMbcnt:                  out.debug << "mbcnt";                       break;

        case EOpFragmentMaskFetch:      out.debug << "fragmentMaskFetchAMD";        break;
        case EOpFragmentFetch:          out.debug << "fragmentFetchAMD";            break;

        case EOpCubeFaceIndex:          out.debug << "cubeFaceIndex";               break;
        case EOpCubeFaceCoord:          out.debug << "cubeFaceCoord";               break;

        case EOpSubpassLoad:   out.debug << "subpassLoad";   break;
        case EOpSubpassLoadMS: out.debug << "subpassLoadMS"; break;

        case EOpConstructReference: out.debug << "Construct reference type"; break;

#ifndef GLSLANG_WEB
        case EOpSpirvInst: out.debug << "spirv_instruction"; break;
#endif

        default: out.debug.message(EPrefixError, "Bad unary op");
        }
    }
    else if (visit == EvPostVisit) {
        switch (node->getOp()) {
        case EOpNegative:
        case EOpLogicalNot:
        case EOpBitwiseNot:

        case EOpPreIncrement:
        case EOpPreDecrement:
        case EOpPostIncrement:
        case EOpPostDecrement: out.debug << ""; break;

        default: out.debug << ")";
        }
    }

    return true;
}

bool TConvertTraverser::visitAggregate(TVisit visit, TIntermAggregate* node)
{
    TInfoSink& out = infoSink;

    if (node->getOp() == EOpNull) {
        out.debug.message(EPrefixError, "node is still EOpNull!");
        return true;
    }

    if (visit == EvPreVisit) {
        tryNewLine(node);
        switch (node->getOp()) {
        case EOpSequence: {
            const char* sep = sequenceSeperator.top();
            sequenceSeperator.push(sep);
            sequenceEnd.push("");
            break;
        }
        case EOpLinkerObjects: declaringSymbol = 1; break;
        case EOpComma:         out.debug << "Comma";            break;
        case EOpFunction: {
            std::string::size_type n = node->getName().find("(");
            if (n != std::string::npos) {
                out.debug << node->getTypeString() << " " << node->getName().substr(0, n);
            }
            else {
                out.debug << node->getTypeString() << " " << node->getName();
            }
            sequenceSeperator.push(";");
            sequenceEnd.push(";");
            localSymbols.clear();
            level++;
            break;
        }
        case EOpFunctionCall: {
            std::string::size_type n = node->getName().find("(");
            if (n != std::string::npos) {
                out.debug << node->getName().substr(0, n);
            }
            else {
                out.debug << node->getName();
            }
            out.debug << "(";
            break;
        }
        case EOpParameters: {
            declaringSymbol = 2;
            out.debug << "(";
            break;
        }

        case EOpConstructFloat:
        case EOpConstructDouble:

        case EOpConstructVec2:
        case EOpConstructVec3:
        case EOpConstructVec4:
        case EOpConstructDVec2:
        case EOpConstructDVec3:
        case EOpConstructDVec4:
        case EOpConstructBool:
        case EOpConstructBVec2:
        case EOpConstructBVec3:
        case EOpConstructBVec4:
        case EOpConstructInt8:
        case EOpConstructI8Vec2:
        case EOpConstructI8Vec3:
        case EOpConstructI8Vec4:
        case EOpConstructInt:
        case EOpConstructIVec2:
        case EOpConstructIVec3:
        case EOpConstructIVec4:
        case EOpConstructUint8:
        case EOpConstructU8Vec2:
        case EOpConstructU8Vec3:
        case EOpConstructU8Vec4:
        case EOpConstructUint:
        case EOpConstructUVec2:
        case EOpConstructUVec3:
        case EOpConstructUVec4:
        case EOpConstructInt64:
        case EOpConstructI64Vec2:
        case EOpConstructI64Vec3:
        case EOpConstructI64Vec4:
        case EOpConstructUint64:
        case EOpConstructU64Vec2:
        case EOpConstructU64Vec3:
        case EOpConstructU64Vec4:
        case EOpConstructInt16:
        case EOpConstructI16Vec2:
        case EOpConstructI16Vec3:
        case EOpConstructI16Vec4:
        case EOpConstructUint16:
        case EOpConstructU16Vec2:
        case EOpConstructU16Vec3:
        case EOpConstructU16Vec4:
        case EOpConstructMat2x2:
        case EOpConstructMat2x3:
        case EOpConstructMat2x4:
        case EOpConstructMat3x2:
        case EOpConstructMat3x3:
        case EOpConstructMat3x4:
        case EOpConstructMat4x2:
        case EOpConstructMat4x3:
        case EOpConstructMat4x4:
        case EOpConstructDMat2x2:
        case EOpConstructDMat2x3:
        case EOpConstructDMat2x4:
        case EOpConstructDMat3x2:
        case EOpConstructDMat3x3:
        case EOpConstructDMat3x4:
        case EOpConstructDMat4x2:
        case EOpConstructDMat4x3:
        case EOpConstructDMat4x4:
        case EOpConstructIMat2x2:
        case EOpConstructIMat2x3:
        case EOpConstructIMat2x4:
        case EOpConstructIMat3x2:
        case EOpConstructIMat3x3:
        case EOpConstructIMat3x4:
        case EOpConstructIMat4x2:
        case EOpConstructIMat4x3:
        case EOpConstructIMat4x4:
        case EOpConstructUMat2x2:
        case EOpConstructUMat2x3:
        case EOpConstructUMat2x4:
        case EOpConstructUMat3x2:
        case EOpConstructUMat3x3:
        case EOpConstructUMat3x4:
        case EOpConstructUMat4x2:
        case EOpConstructUMat4x3:
        case EOpConstructUMat4x4:
        case EOpConstructBMat2x2:
        case EOpConstructBMat2x3:
        case EOpConstructBMat2x4:
        case EOpConstructBMat3x2:
        case EOpConstructBMat3x3:
        case EOpConstructBMat3x4:
        case EOpConstructBMat4x2:
        case EOpConstructBMat4x3:
        case EOpConstructBMat4x4:
        case EOpConstructFloat16:
        case EOpConstructF16Vec2:
        case EOpConstructF16Vec3:
        case EOpConstructF16Vec4:
        case EOpConstructF16Mat2x2:
        case EOpConstructF16Mat2x3:
        case EOpConstructF16Mat2x4:
        case EOpConstructF16Mat3x2:
        case EOpConstructF16Mat3x3:
        case EOpConstructF16Mat3x4:
        case EOpConstructF16Mat4x2:
        case EOpConstructF16Mat4x3:
        case EOpConstructF16Mat4x4:
        case EOpConstructStruct: out.debug << node->getTypeString() << "(";  break;
        case EOpConstructTextureSampler: out.debug << "Construct combined texture-sampler"; break;
        case EOpConstructReference:  out.debug << "Construct reference";  break;
        case EOpConstructCooperativeMatrix:  out.debug << "Construct cooperative matrix";  break;
        case EOpConstructAccStruct: out.debug << "Construct acceleration structure"; break;

        case EOpLessThan:         out.debug << "lessThan(";             break;
        case EOpGreaterThan:      out.debug << "greaterThan(";          break;
        case EOpLessThanEqual:    out.debug << "LessThanEqual(";    break;
        case EOpGreaterThanEqual: out.debug << "greaterThanEqual("; break;
        case EOpVectorEqual:      out.debug << "equal(";                         break;
        case EOpVectorNotEqual:   out.debug << "notEqual(";                      break;

        case EOpMod:           out.debug << "mod(";         break;
        case EOpModf:          out.debug << "modf(";        break;
        case EOpPow:           out.debug << "pow(";         break;

        case EOpAtan:          out.debug << "atan("; break;

        case EOpMin:           out.debug << "min(";         break;
        case EOpMax:           out.debug << "max(";         break;
        case EOpClamp:         out.debug << "clamp(";       break;
        case EOpMix:           out.debug << "mix(";         break;
        case EOpStep:          out.debug << "step(";        break;
        case EOpSmoothStep:    out.debug << "smoothstep(";  break;

        case EOpDistance:      out.debug << "distance(";                break;
        case EOpDot:           out.debug << "dot(";             break;
        case EOpCross:         out.debug << "cross(";           break;
        case EOpFaceForward:   out.debug << "faceforward(";            break;
        case EOpReflect:       out.debug << "reflect(";                 break;
        case EOpRefract:       out.debug << "refract(";                 break;
        case EOpMul:           out.debug << "matrixCompMult("; break;
        case EOpOuterProduct:  out.debug << "outerProduct(";           break;

        case EOpEmitVertex:    out.debug << "EmitVertex";              break;
        case EOpEndPrimitive:  out.debug << "EndPrimitive";            break;

        case EOpBarrier:                    out.debug << "Barrier";                    break;
        case EOpMemoryBarrier:              out.debug << "MemoryBarrier";              break;
        case EOpMemoryBarrierAtomicCounter: out.debug << "MemoryBarrierAtomicCounter"; break;
        case EOpMemoryBarrierBuffer:        out.debug << "MemoryBarrierBuffer";        break;
        case EOpMemoryBarrierImage:         out.debug << "MemoryBarrierImage";         break;
        case EOpMemoryBarrierShared:        out.debug << "MemoryBarrierShared";        break;
        case EOpGroupMemoryBarrier:         out.debug << "GroupMemoryBarrier";         break;

        case EOpReadInvocation:             out.debug << "readInvocation";        break;

        case EOpSwizzleInvocations:         out.debug << "swizzleInvocations";       break;
        case EOpSwizzleInvocationsMasked:   out.debug << "swizzleInvocationsMasked"; break;
        case EOpWriteInvocation:            out.debug << "writeInvocation";          break;

        case EOpMin3:                       out.debug << "min3";                  break;
        case EOpMax3:                       out.debug << "max3";                  break;
        case EOpMid3:                       out.debug << "mid3";                  break;
        case EOpTime:                       out.debug << "time";                  break;

        case EOpAtomicAdd:                  out.debug << "AtomicAdd";             break;
        case EOpAtomicSubtract:             out.debug << "AtomicSubtract";        break;
        case EOpAtomicMin:                  out.debug << "AtomicMin";             break;
        case EOpAtomicMax:                  out.debug << "AtomicMax";             break;
        case EOpAtomicAnd:                  out.debug << "AtomicAnd";             break;
        case EOpAtomicOr:                   out.debug << "AtomicOr";              break;
        case EOpAtomicXor:                  out.debug << "AtomicXor";             break;
        case EOpAtomicExchange:             out.debug << "AtomicExchange";        break;
        case EOpAtomicCompSwap:             out.debug << "AtomicCompSwap";        break;
        case EOpAtomicLoad:                 out.debug << "AtomicLoad";            break;
        case EOpAtomicStore:                out.debug << "AtomicStore";           break;

        case EOpAtomicCounterAdd:           out.debug << "AtomicCounterAdd";      break;
        case EOpAtomicCounterSubtract:      out.debug << "AtomicCounterSubtract"; break;
        case EOpAtomicCounterMin:           out.debug << "AtomicCounterMin";      break;
        case EOpAtomicCounterMax:           out.debug << "AtomicCounterMax";      break;
        case EOpAtomicCounterAnd:           out.debug << "AtomicCounterAnd";      break;
        case EOpAtomicCounterOr:            out.debug << "AtomicCounterOr";       break;
        case EOpAtomicCounterXor:           out.debug << "AtomicCounterXor";      break;
        case EOpAtomicCounterExchange:      out.debug << "AtomicCounterExchange"; break;
        case EOpAtomicCounterCompSwap:      out.debug << "AtomicCounterCompSwap"; break;

        case EOpImageQuerySize:             out.debug << "imageQuerySize";        break;
        case EOpImageQuerySamples:          out.debug << "imageQuerySamples";     break;
        case EOpImageLoad:                  out.debug << "imageLoad";             break;
        case EOpImageStore:                 out.debug << "imageStore";            break;
        case EOpImageAtomicAdd:             out.debug << "imageAtomicAdd";        break;
        case EOpImageAtomicMin:             out.debug << "imageAtomicMin";        break;
        case EOpImageAtomicMax:             out.debug << "imageAtomicMax";        break;
        case EOpImageAtomicAnd:             out.debug << "imageAtomicAnd";        break;
        case EOpImageAtomicOr:              out.debug << "imageAtomicOr";         break;
        case EOpImageAtomicXor:             out.debug << "imageAtomicXor";        break;
        case EOpImageAtomicExchange:        out.debug << "imageAtomicExchange";   break;
        case EOpImageAtomicCompSwap:        out.debug << "imageAtomicCompSwap";   break;
        case EOpImageAtomicLoad:            out.debug << "imageAtomicLoad";       break;
        case EOpImageAtomicStore:           out.debug << "imageAtomicStore";      break;
        case EOpImageLoadLod:               out.debug << "imageLoadLod";          break;
        case EOpImageStoreLod:              out.debug << "imageStoreLod";         break;

        case EOpTextureQuerySize:           out.debug << "textureSize(";           break;
        case EOpTextureQueryLod:            out.debug << "textureQueryLod";       break;
        case EOpTextureQueryLevels:         out.debug << "textureQueryLevels";    break;
        case EOpTextureQuerySamples:        out.debug << "textureSamples";        break;
        case EOpTexture:                    out.debug << "texture(";               break;
        case EOpTextureProj:                out.debug << "textureProj(";           break;
        case EOpTextureLod:                 out.debug << "textureLod(";            break;
        case EOpTextureOffset:              out.debug << "textureOffset(";         break;
        case EOpTextureFetch:               out.debug << "texelFetch(";          break;
        case EOpTextureFetchOffset:         out.debug << "texelFetchOffset(";    break;
        case EOpTextureProjOffset:          out.debug << "textureProjOffset(";     break;
        case EOpTextureLodOffset:           out.debug << "textureLodOffset(";      break;
        case EOpTextureProjLod:             out.debug << "textureProjLod(";        break;
        case EOpTextureProjLodOffset:       out.debug << "textureProjLodOffset(";  break;
        case EOpTextureGrad:                out.debug << "textureGrad(";           break;
        case EOpTextureGradOffset:          out.debug << "textureGradOffset(";     break;
        case EOpTextureProjGrad:            out.debug << "textureProjGrad(";       break;
        case EOpTextureProjGradOffset:      out.debug << "textureProjGradOffset("; break;
        case EOpTextureGather:              out.debug << "textureGather";         break;
        case EOpTextureGatherOffset:        out.debug << "textureGatherOffset";   break;
        case EOpTextureGatherOffsets:       out.debug << "textureGatherOffsets";  break;
        case EOpTextureClamp:               out.debug << "textureClamp";          break;
        case EOpTextureOffsetClamp:         out.debug << "textureOffsetClamp";    break;
        case EOpTextureGradClamp:           out.debug << "textureGradClamp";      break;
        case EOpTextureGradOffsetClamp:     out.debug << "textureGradOffsetClamp";  break;
        case EOpTextureGatherLod:           out.debug << "textureGatherLod";        break;
        case EOpTextureGatherLodOffset:     out.debug << "textureGatherLodOffset";  break;
        case EOpTextureGatherLodOffsets:    out.debug << "textureGatherLodOffsets"; break;

        case EOpSparseTexture:                  out.debug << "sparseTexture";                   break;
        case EOpSparseTextureOffset:            out.debug << "sparseTextureOffset";             break;
        case EOpSparseTextureLod:               out.debug << "sparseTextureLod";                break;
        case EOpSparseTextureLodOffset:         out.debug << "sparseTextureLodOffset";          break;
        case EOpSparseTextureFetch:             out.debug << "sparseTexelFetch";                break;
        case EOpSparseTextureFetchOffset:       out.debug << "sparseTexelFetchOffset";          break;
        case EOpSparseTextureGrad:              out.debug << "sparseTextureGrad";               break;
        case EOpSparseTextureGradOffset:        out.debug << "sparseTextureGradOffset";         break;
        case EOpSparseTextureGather:            out.debug << "sparseTextureGather";             break;
        case EOpSparseTextureGatherOffset:      out.debug << "sparseTextureGatherOffset";       break;
        case EOpSparseTextureGatherOffsets:     out.debug << "sparseTextureGatherOffsets";      break;
        case EOpSparseImageLoad:                out.debug << "sparseImageLoad";                 break;
        case EOpSparseTextureClamp:             out.debug << "sparseTextureClamp";              break;
        case EOpSparseTextureOffsetClamp:       out.debug << "sparseTextureOffsetClamp";        break;
        case EOpSparseTextureGradClamp:         out.debug << "sparseTextureGradClamp";          break;
        case EOpSparseTextureGradOffsetClamp:   out.debug << "sparseTextureGradOffsetClam";     break;
        case EOpSparseTextureGatherLod:         out.debug << "sparseTextureGatherLod";          break;
        case EOpSparseTextureGatherLodOffset:   out.debug << "sparseTextureGatherLodOffset";    break;
        case EOpSparseTextureGatherLodOffsets:  out.debug << "sparseTextureGatherLodOffsets";   break;
        case EOpSparseImageLoadLod:             out.debug << "sparseImageLoadLod";              break;
        case EOpImageSampleFootprintNV:             out.debug << "imageSampleFootprintNV";          break;
        case EOpImageSampleFootprintClampNV:        out.debug << "imageSampleFootprintClampNV";     break;
        case EOpImageSampleFootprintLodNV:          out.debug << "imageSampleFootprintLodNV";       break;
        case EOpImageSampleFootprintGradNV:         out.debug << "imageSampleFootprintGradNV";      break;
        case EOpImageSampleFootprintGradClampNV:    out.debug << "mageSampleFootprintGradClampNV";  break;
        case EOpAddCarry:                   out.debug << "addCarry";              break;
        case EOpSubBorrow:                  out.debug << "subBorrow";             break;
        case EOpUMulExtended:               out.debug << "uMulExtended";          break;
        case EOpIMulExtended:               out.debug << "iMulExtended";          break;
        case EOpBitfieldExtract:            out.debug << "bitfieldExtract";       break;
        case EOpBitfieldInsert:             out.debug << "bitfieldInsert";        break;

        case EOpFma:                        out.debug << "fma";                   break;
        case EOpFrexp:                      out.debug << "frexp";                 break;
        case EOpLdexp:                      out.debug << "ldexp";                 break;

        case EOpInterpolateAtSample:   out.debug << "interpolateAtSample";    break;
        case EOpInterpolateAtOffset:   out.debug << "interpolateAtOffset";    break;
        case EOpInterpolateAtVertex:   out.debug << "interpolateAtVertex";    break;

        case EOpSinCos:                     out.debug << "sincos";                break;
        case EOpGenMul:                     out.debug << "mul";                   break;

        case EOpAllMemoryBarrierWithGroupSync:    out.debug << "AllMemoryBarrierWithGroupSync";    break;
        case EOpDeviceMemoryBarrier:              out.debug << "DeviceMemoryBarrier";              break;
        case EOpDeviceMemoryBarrierWithGroupSync: out.debug << "DeviceMemoryBarrierWithGroupSync"; break;
        case EOpWorkgroupMemoryBarrier:           out.debug << "WorkgroupMemoryBarrier";           break;
        case EOpWorkgroupMemoryBarrierWithGroupSync: out.debug << "WorkgroupMemoryBarrierWithGroupSync"; break;

        case EOpSubgroupBarrier:                 out.debug << "subgroupBarrier"; break;
        case EOpSubgroupMemoryBarrier:           out.debug << "subgroupMemoryBarrier"; break;
        case EOpSubgroupMemoryBarrierBuffer:     out.debug << "subgroupMemoryBarrierBuffer"; break;
        case EOpSubgroupMemoryBarrierImage:      out.debug << "subgroupMemoryBarrierImage";   break;
        case EOpSubgroupMemoryBarrierShared:     out.debug << "subgroupMemoryBarrierShared"; break;
        case EOpSubgroupElect:                   out.debug << "subgroupElect"; break;
        case EOpSubgroupAll:                     out.debug << "subgroupAll"; break;
        case EOpSubgroupAny:                     out.debug << "subgroupAny"; break;
        case EOpSubgroupAllEqual:                out.debug << "subgroupAllEqual"; break;
        case EOpSubgroupBroadcast:               out.debug << "subgroupBroadcast"; break;
        case EOpSubgroupBroadcastFirst:          out.debug << "subgroupBroadcastFirst"; break;
        case EOpSubgroupBallot:                  out.debug << "subgroupBallot"; break;
        case EOpSubgroupInverseBallot:           out.debug << "subgroupInverseBallot"; break;
        case EOpSubgroupBallotBitExtract:        out.debug << "subgroupBallotBitExtract"; break;
        case EOpSubgroupBallotBitCount:          out.debug << "subgroupBallotBitCount"; break;
        case EOpSubgroupBallotInclusiveBitCount: out.debug << "subgroupBallotInclusiveBitCount"; break;
        case EOpSubgroupBallotExclusiveBitCount: out.debug << "subgroupBallotExclusiveBitCount"; break;
        case EOpSubgroupBallotFindLSB:           out.debug << "subgroupBallotFindLSB"; break;
        case EOpSubgroupBallotFindMSB:           out.debug << "subgroupBallotFindMSB"; break;
        case EOpSubgroupShuffle:                 out.debug << "subgroupShuffle"; break;
        case EOpSubgroupShuffleXor:              out.debug << "subgroupShuffleXor"; break;
        case EOpSubgroupShuffleUp:               out.debug << "subgroupShuffleUp"; break;
        case EOpSubgroupShuffleDown:             out.debug << "subgroupShuffleDown"; break;
        case EOpSubgroupAdd:                     out.debug << "subgroupAdd"; break;
        case EOpSubgroupMul:                     out.debug << "subgroupMul"; break;
        case EOpSubgroupMin:                     out.debug << "subgroupMin"; break;
        case EOpSubgroupMax:                     out.debug << "subgroupMax"; break;
        case EOpSubgroupAnd:                     out.debug << "subgroupAnd"; break;
        case EOpSubgroupOr:                      out.debug << "subgroupOr"; break;
        case EOpSubgroupXor:                     out.debug << "subgroupXor"; break;
        case EOpSubgroupInclusiveAdd:            out.debug << "subgroupInclusiveAdd"; break;
        case EOpSubgroupInclusiveMul:            out.debug << "subgroupInclusiveMul"; break;
        case EOpSubgroupInclusiveMin:            out.debug << "subgroupInclusiveMin"; break;
        case EOpSubgroupInclusiveMax:            out.debug << "subgroupInclusiveMax"; break;
        case EOpSubgroupInclusiveAnd:            out.debug << "subgroupInclusiveAnd"; break;
        case EOpSubgroupInclusiveOr:             out.debug << "subgroupInclusiveOr"; break;
        case EOpSubgroupInclusiveXor:            out.debug << "subgroupInclusiveXor"; break;
        case EOpSubgroupExclusiveAdd:            out.debug << "subgroupExclusiveAdd"; break;
        case EOpSubgroupExclusiveMul:            out.debug << "subgroupExclusiveMul"; break;
        case EOpSubgroupExclusiveMin:            out.debug << "subgroupExclusiveMin"; break;
        case EOpSubgroupExclusiveMax:            out.debug << "subgroupExclusiveMax"; break;
        case EOpSubgroupExclusiveAnd:            out.debug << "subgroupExclusiveAnd"; break;
        case EOpSubgroupExclusiveOr:             out.debug << "subgroupExclusiveOr"; break;
        case EOpSubgroupExclusiveXor:            out.debug << "subgroupExclusiveXor"; break;
        case EOpSubgroupClusteredAdd:            out.debug << "subgroupClusteredAdd"; break;
        case EOpSubgroupClusteredMul:            out.debug << "subgroupClusteredMul"; break;
        case EOpSubgroupClusteredMin:            out.debug << "subgroupClusteredMin"; break;
        case EOpSubgroupClusteredMax:            out.debug << "subgroupClusteredMax"; break;
        case EOpSubgroupClusteredAnd:            out.debug << "subgroupClusteredAnd"; break;
        case EOpSubgroupClusteredOr:             out.debug << "subgroupClusteredOr"; break;
        case EOpSubgroupClusteredXor:            out.debug << "subgroupClusteredXor"; break;
        case EOpSubgroupQuadBroadcast:           out.debug << "subgroupQuadBroadcast"; break;
        case EOpSubgroupQuadSwapHorizontal:      out.debug << "subgroupQuadSwapHorizontal"; break;
        case EOpSubgroupQuadSwapVertical:        out.debug << "subgroupQuadSwapVertical"; break;
        case EOpSubgroupQuadSwapDiagonal:        out.debug << "subgroupQuadSwapDiagonal"; break;

        case EOpSubgroupPartition:                          out.debug << "subgroupPartitionNV";                          break;
        case EOpSubgroupPartitionedAdd:                     out.debug << "subgroupPartitionedAddNV";                     break;
        case EOpSubgroupPartitionedMul:                     out.debug << "subgroupPartitionedMulNV";                     break;
        case EOpSubgroupPartitionedMin:                     out.debug << "subgroupPartitionedMinNV";                     break;
        case EOpSubgroupPartitionedMax:                     out.debug << "subgroupPartitionedMaxNV";                     break;
        case EOpSubgroupPartitionedAnd:                     out.debug << "subgroupPartitionedAndNV";                     break;
        case EOpSubgroupPartitionedOr:                      out.debug << "subgroupPartitionedOrNV";                      break;
        case EOpSubgroupPartitionedXor:                     out.debug << "subgroupPartitionedXorNV";                     break;
        case EOpSubgroupPartitionedInclusiveAdd:            out.debug << "subgroupPartitionedInclusiveAddNV";            break;
        case EOpSubgroupPartitionedInclusiveMul:            out.debug << "subgroupPartitionedInclusiveMulNV";            break;
        case EOpSubgroupPartitionedInclusiveMin:            out.debug << "subgroupPartitionedInclusiveMinNV";            break;
        case EOpSubgroupPartitionedInclusiveMax:            out.debug << "subgroupPartitionedInclusiveMaxNV";            break;
        case EOpSubgroupPartitionedInclusiveAnd:            out.debug << "subgroupPartitionedInclusiveAndNV";            break;
        case EOpSubgroupPartitionedInclusiveOr:             out.debug << "subgroupPartitionedInclusiveOrNV";             break;
        case EOpSubgroupPartitionedInclusiveXor:            out.debug << "subgroupPartitionedInclusiveXorNV";            break;
        case EOpSubgroupPartitionedExclusiveAdd:            out.debug << "subgroupPartitionedExclusiveAddNV";            break;
        case EOpSubgroupPartitionedExclusiveMul:            out.debug << "subgroupPartitionedExclusiveMulNV";            break;
        case EOpSubgroupPartitionedExclusiveMin:            out.debug << "subgroupPartitionedExclusiveMinNV";            break;
        case EOpSubgroupPartitionedExclusiveMax:            out.debug << "subgroupPartitionedExclusiveMaxNV";            break;
        case EOpSubgroupPartitionedExclusiveAnd:            out.debug << "subgroupPartitionedExclusiveAndNV";            break;
        case EOpSubgroupPartitionedExclusiveOr:             out.debug << "subgroupPartitionedExclusiveOrNV";             break;
        case EOpSubgroupPartitionedExclusiveXor:            out.debug << "subgroupPartitionedExclusiveXorNV";            break;

        case EOpSubpassLoad:   out.debug << "subpassLoad";   break;
        case EOpSubpassLoadMS: out.debug << "subpassLoadMS"; break;

        case EOpTraceNV:                          out.debug << "traceNV"; break;
        case EOpTraceRayMotionNV:                 out.debug << "traceRayMotionNV"; break;
        case EOpTraceKHR:                         out.debug << "traceRayKHR"; break;
        case EOpReportIntersection:               out.debug << "reportIntersectionNV"; break;
        case EOpIgnoreIntersectionNV:             out.debug << "ignoreIntersectionNV"; break;
        case EOpIgnoreIntersectionKHR:            out.debug << "ignoreIntersectionKHR"; break;
        case EOpTerminateRayNV:                   out.debug << "terminateRayNV"; break;
        case EOpTerminateRayKHR:                  out.debug << "terminateRayKHR"; break;
        case EOpExecuteCallableNV:                out.debug << "executeCallableNV"; break;
        case EOpExecuteCallableKHR:               out.debug << "executeCallableKHR"; break;
        case EOpWritePackedPrimitiveIndices4x8NV: out.debug << "writePackedPrimitiveIndices4x8NV"; break;

        case EOpRayQueryInitialize:                                            out.debug << "rayQueryInitializeEXT"; break;
        case EOpRayQueryTerminate:                                             out.debug << "rayQueryTerminateEXT"; break;
        case EOpRayQueryGenerateIntersection:                                  out.debug << "rayQueryGenerateIntersectionEXT"; break;
        case EOpRayQueryConfirmIntersection:                                   out.debug << "rayQueryConfirmIntersectionEXT"; break;
        case EOpRayQueryProceed:                                               out.debug << "rayQueryProceedEXT"; break;
        case EOpRayQueryGetIntersectionType:                                   out.debug << "rayQueryGetIntersectionTypeEXT"; break;
        case EOpRayQueryGetRayTMin:                                            out.debug << "rayQueryGetRayTMinEXT"; break;
        case EOpRayQueryGetRayFlags:                                           out.debug << "rayQueryGetRayFlagsEXT"; break;
        case EOpRayQueryGetIntersectionT:                                      out.debug << "rayQueryGetIntersectionTEXT"; break;
        case EOpRayQueryGetIntersectionInstanceCustomIndex:                    out.debug << "rayQueryGetIntersectionInstanceCustomIndexEXT"; break;
        case EOpRayQueryGetIntersectionInstanceId:                             out.debug << "rayQueryGetIntersectionInstanceIdEXT"; break;
        case EOpRayQueryGetIntersectionInstanceShaderBindingTableRecordOffset: out.debug << "rayQueryGetIntersectionInstanceShaderBindingTableRecordOffsetEXT"; break;
        case EOpRayQueryGetIntersectionGeometryIndex:                          out.debug << "rayQueryGetIntersectionGeometryIndexEXT"; break;
        case EOpRayQueryGetIntersectionPrimitiveIndex:                         out.debug << "rayQueryGetIntersectionPrimitiveIndexEXT"; break;
        case EOpRayQueryGetIntersectionBarycentrics:                           out.debug << "rayQueryGetIntersectionBarycentricsEXT"; break;
        case EOpRayQueryGetIntersectionFrontFace:                              out.debug << "rayQueryGetIntersectionFrontFaceEXT"; break;
        case EOpRayQueryGetIntersectionCandidateAABBOpaque:                    out.debug << "rayQueryGetIntersectionCandidateAABBOpaqueEXT"; break;
        case EOpRayQueryGetIntersectionObjectRayDirection:                     out.debug << "rayQueryGetIntersectionObjectRayDirectionEXT"; break;
        case EOpRayQueryGetIntersectionObjectRayOrigin:                        out.debug << "rayQueryGetIntersectionObjectRayOriginEXT"; break;
        case EOpRayQueryGetWorldRayDirection:                                  out.debug << "rayQueryGetWorldRayDirectionEXT"; break;
        case EOpRayQueryGetWorldRayOrigin:                                     out.debug << "rayQueryGetWorldRayOriginEXT"; break;
        case EOpRayQueryGetIntersectionObjectToWorld:                          out.debug << "rayQueryGetIntersectionObjectToWorldEXT"; break;
        case EOpRayQueryGetIntersectionWorldToObject:                          out.debug << "rayQueryGetIntersectionWorldToObjectEXT"; break;

        case EOpCooperativeMatrixLoad:  out.debug << "Load cooperative matrix";  break;
        case EOpCooperativeMatrixStore:  out.debug << "Store cooperative matrix";  break;
        case EOpCooperativeMatrixMulAdd: out.debug << "MulAdd cooperative matrices"; break;

        case EOpIsHelperInvocation: out.debug << "IsHelperInvocation"; break;
        case EOpDebugPrintf:  out.debug << "Debug printf";  break;

#ifndef GLSLANG_WEB
        case EOpSpirvInst: out.debug << "spirv_instruction"; break;
#endif

        default: out.debug.message(EPrefixError, "Bad aggregation op");
        }
    }
    else if (visit == EvInVisit) {
        switch (node->getOp()) {
        case EOpLinkerObjects: {
            if (!skipped) {
                out.debug << ";";
            }
            else {
                skipped = false;
            }
            return true;
        }
        case EOpSequence: {
            if (!skipped) {
                out.debug << sequenceSeperator.top();
            }
            else {
                skipped = false;
            }
            break;
        }
        case EOpFunction: break;
        default: out.debug << ", ";
        }
    }
    else if (visit == EvPostVisit) {
        switch (node->getOp()) {
        case EOpLinkerObjects: {
            declaringSymbol = 0;
            if (!skipped) {
                out.debug << ";";
            }
            else {
                skipped = false;
            }
            return true;
        }
        case EOpSequence: {
            sequenceSeperator.pop();
            sequenceEnd.pop();
            if (!skipped) {
                out.debug << sequenceEnd.top();
            }
            else {
                skipped = false;
            }
            break;
        }
        case EOpFunction: {
            level--;
            gotoNewLine();
            out.debug << "}";
            sequenceSeperator.pop();
            sequenceEnd.pop();
            break;
        }
        case EOpParameters: {
            declaringSymbol = 0;
            out.debug << ") {";
            break;
        }

        default: out.debug << ")";
        }
    }
    return true;
}

bool TConvertTraverser::visitSelection(TVisit /* visit */, TIntermSelection* node)
{
    TInfoSink& out = infoSink;

    out.debug << "Test condition and select";
    out.debug << " (" << node->getCompleteString() << ")";

    if (node->getShortCircuit() == false)
        out.debug << ": no shortcircuit";
    if (node->getFlatten())
        out.debug << ": Flatten";
    if (node->getDontFlatten())
        out.debug << ": DontFlatten";
    out.debug << "\n";

    ++depth;

    out.debug << "Condition\n";
    node->getCondition()->traverse(this);

    if (node->getTrueBlock()) {
        out.debug << "true case\n";
        node->getTrueBlock()->traverse(this);
    } else
        out.debug << "true case is null\n";

    if (node->getFalseBlock()) {
        out.debug << "false case\n";
        node->getFalseBlock()->traverse(this);
    }

    --depth;

    return false;
}

// Print infinities and NaNs, and numbers in a portable way.
// Goals:
//   - portable (across IEEE 754 platforms)
//   - shows all possible IEEE values
//   - shows simple numbers in a simple way, e.g., no leading/trailing 0s
//   - shows all digits, no premature rounding
static void OutputDouble(TInfoSink& out, double value, TConvertTraverser::EExtraOutput extra)
{
    if (IsInfinity(value)) {
        if (value < 0)
            out.debug << "-1.#INF";
        else
            out.debug << "+1.#INF";
    } else if (IsNan(value))
        out.debug << "1.#IND";
    else {
        const int maxSize = 340;
        char buf[maxSize];
        const char* format = "%f";
        if (fabs(value) > 0.0 && (fabs(value) < 1e-5 || fabs(value) > 1e12))
            format = "%-.13e";
        int len = snprintf(buf, maxSize, format, value);
        assert(len < maxSize);

        // remove a leading zero in the 100s slot in exponent; it is not portable
        // pattern:   XX...XXXe+0XX or XX...XXXe-0XX
        if (len > 5) {
            if (buf[len-5] == 'e' && (buf[len-4] == '+' || buf[len-4] == '-') && buf[len-3] == '0') {
                buf[len-3] = buf[len-2];
                buf[len-2] = buf[len-1];
                buf[len-1] = '\0';
            }
        }

        out.debug << buf;

        switch (extra) {
        case TConvertTraverser::BinaryDoubleOutput:
        {
            uint64_t b;
            static_assert(sizeof(b) == sizeof(value), "sizeof(uint64_t) != sizeof(double)");
            memcpy(&b, &value, sizeof(b));

            out.debug << " : ";
            for (size_t i = 0; i < 8 * sizeof(value); ++i, ++b) {
                out.debug << ((b & 0x8000000000000000) != 0 ? "1" : "0");
                b <<= 1;
            }
            break;
        }
        default:
            break;
        }
    }
}

static void OutputConstantUnion(TInfoSink& out, const TIntermTyped* node, const TConstUnionArray& constUnion,
    TConvertTraverser::EExtraOutput extra, bool swizzling)
{
    int size = node->getType().computeNumComponents();

    int arraySize = 0;
    int elementSize = 0;
    if (node->getType().isArray()) {
        arraySize = node->getType().getOuterArraySize();
        elementSize = size / arraySize;
        out.debug << node->getTypeString() << "(";
    }
    else {
        arraySize = 1;
        elementSize = size;
    }
    bool arrayNeedComma = false;
    for (int j = 0; j < arraySize; j++) {
        if (arrayNeedComma) out.debug << ", ";
        if (elementSize > 1) {
            out.debug << node->getType().getArrayElementTypeString() << "(";
        }
        bool elementNeedComma = false;
        for (int i = 0; i < elementSize; i++) {
            if (elementNeedComma) out.debug << ", ";
            switch (constUnion[i].getType()) {
            case EbtBool:
                if (constUnion[i].getBConst())
                    out.debug << "true";
                else
                    out.debug << "false";

                break;
            case EbtFloat:
            case EbtDouble:
            case EbtFloat16:
                OutputDouble(out, constUnion[i].getDConst(), extra);
                break;
            case EbtInt8:
                {
                    const int maxSize = 300;
                    char buf[maxSize];
                    snprintf(buf, maxSize, "%d", constUnion[i].getI8Const());

                    out.debug << buf;
                }
                break;
            case EbtUint8:
                {
                    const int maxSize = 300;
                    char buf[maxSize];
                    snprintf(buf, maxSize, "%u", constUnion[i].getU8Const());

                    out.debug << buf;
                }
                break;
            case EbtInt16:
                {
                    const int maxSize = 300;
                    char buf[maxSize];
                    snprintf(buf, maxSize, "%d", constUnion[i].getI16Const());

                    out.debug << buf;
                }
                break;
            case EbtUint16:
                {
                    const int maxSize = 300;
                    char buf[maxSize];
                    snprintf(buf, maxSize, "%u", constUnion[i].getU16Const());

                    out.debug << buf;
                }
                break;
            case EbtInt:
                {
                    const int maxSize = 300;
                    char buf[maxSize];
                    char names[4] = {'x', 'y', 'z', 'w'};
                    if (swizzling && constUnion[i].getIConst() < 4) {
                        snprintf(buf, maxSize, "%c", names[constUnion[i].getIConst()]);
                    }
                    else {
                        snprintf(buf, maxSize, "%d", constUnion[i].getIConst());
                    }

                    out.debug << buf;
                }
                break;
            case EbtUint:
                {
                    const int maxSize = 300;
                    char buf[maxSize];
                    snprintf(buf, maxSize, "%u", constUnion[i].getUConst());

                    out.debug << buf;
                }
                break;
            case EbtInt64:
                {
                    const int maxSize = 300;
                    char buf[maxSize];
                    snprintf(buf, maxSize, "%lld", constUnion[i].getI64Const());

                    out.debug << buf;
                }
                break;
            case EbtUint64:
                {
                    const int maxSize = 300;
                    char buf[maxSize];
                    snprintf(buf, maxSize, "%llu", constUnion[i].getU64Const());

                    out.debug << buf;
                }
                break;
            case EbtString:
                out.debug << "\"" << constUnion[i].getSConst()->c_str() << "\"";
                break;
            default:
                out.info.message(EPrefixInternalError, "Unknown constant", node->getLoc());
                break;
            }
            elementNeedComma = true;
        }
        if (elementSize > 1) {
            out.debug << ")";
        }
        arrayNeedComma = true;
    }
    if (node->getType().isArray()) {
        out.debug << ")";
    }
}

void TConvertTraverser::visitConstantUnion(TIntermConstantUnion* node)
{
    OutputConstantUnion(infoSink, node, node->getConstArray(), extraOutput, swizzling);
}

void TConvertTraverser::visitSymbol(TIntermSymbol* node)
{
    tryNewLine(node);
    if (declaringSymbol == 0) {
        if (node->getName().compare(0, 6, "anon@0") == 0) {
            skipped = true;
        }
        else {
            if (node->getName().compare(0, 3, "gl_") == 0 ||
                globalSymbols.find(node->getName()) != globalSymbols.end() ||
                localSymbols.find(node->getName()) != localSymbols.end()) {
                infoSink.debug << node->getName();
            }
            else {
                infoSink.debug << node->getTypeString() << " " << node->getName();
                localSymbols.insert(node->getName());
            }
            skipped = false;
        }
    }
    else {
        if (node->getName().compare(0, 6, "anon@0") == 0) {
            skipped = true;
        }
        else if (node->getName().compare(0, 3, "gl_") == 0) {
            skipped = true;
        }
        else if (declaringSymbol == 1 && globalSymbols.find(node->getName()) != globalSymbols.end()) {
            skipped = true;
        }
        else {
            if (declaringSymbol == 1) {
                TString qualifier = node->getStorageQualifierString();
                if (qualifier.compare(0, 6, "global") == 0) {
                    infoSink.debug << "";
                }
                else {
                    infoSink.debug << node->getStorageQualifierString() << " ";
                }
            }
            infoSink.debug << node->getTypeString() << " " << node->getName();
            if (declaringSymbol == 1) {
                globalSymbols.insert(node->getName());
            }
            else {
                localSymbols.insert(node->getName());
            }
            skipped = false;
        }
    }

    if (! node->getConstArray().empty())
        OutputConstantUnion(infoSink, node, node->getConstArray(), extraOutput, swizzling);
    else if (node->getConstSubtree()) {
        incrementDepth(node);
        node->getConstSubtree()->traverse(this);
        decrementDepth();
    }
}

bool TConvertTraverser::visitLoop(TVisit /* visit */, TIntermLoop* node)
{
    TInfoSink& out = infoSink;

    out.debug << "Loop with condition ";
    if (! node->testFirst())
        out.debug << "not ";
    out.debug << "tested first";

    if (node->getUnroll())
        out.debug << ": Unroll";
    if (node->getDontUnroll())
        out.debug << ": DontUnroll";
    if (node->getLoopDependency()) {
        out.debug << ": Dependency ";
        out.debug << node->getLoopDependency();
    }
    out.debug << "\n";

    ++depth;

    if (node->getTest()) {
        out.debug << "Loop Condition\n";
        node->getTest()->traverse(this);
    } else
        out.debug << "No loop condition\n";

    if (node->getBody()) {
        out.debug << "Loop Body\n";
        node->getBody()->traverse(this);
    } else
        out.debug << "No loop body\n";

    if (node->getTerminal()) {
        out.debug << "Loop Terminal Expression\n";
        node->getTerminal()->traverse(this);
    }

    --depth;

    return false;
}

bool TConvertTraverser::visitBranch(TVisit /* visit*/, TIntermBranch* node)
{
    TInfoSink& out = infoSink;
    tryNewLine(node);
    switch (node->getFlowOp()) {
    case EOpKill:                   out.debug << "discard ";                      break;
    case EOpTerminateInvocation:    out.debug << "Branch: TerminateInvocation";   break;
    case EOpIgnoreIntersectionKHR:  out.debug << "Branch: IgnoreIntersectionKHR"; break;
    case EOpTerminateRayKHR:        out.debug << "Branch: TerminateRayKHR";       break;
    case EOpBreak:                  out.debug << "break ";                        break;
    case EOpContinue:               out.debug << "continue ";                     break;
    case EOpReturn:                 out.debug << "return ";                       break;
    case EOpCase:                   out.debug << "case: ";                        break;
    case EOpDemote:                 out.debug << "Demote";                        break;
    case EOpDefault:                out.debug << "default: ";                     break;
    default:                        out.debug << "Branch: Unknown Branch";        break;
    }

    if (node->getExpression()) {
        node->getExpression()->traverse(this);
    }

    return false;
}

bool TConvertTraverser::visitSwitch(TVisit /* visit */, TIntermSwitch* node)
{
    TInfoSink& out = infoSink;

    out.debug << "switch";

    if (node->getFlatten())
        out.debug << ": Flatten";
    if (node->getDontFlatten())
        out.debug << ": DontFlatten";
    out.debug << "\n";

    out.debug << "condition\n";
    ++depth;
    node->getCondition()->traverse(this);

    --depth;
    out.debug << "body\n";
    ++depth;
    node->getBody()->traverse(this);

    --depth;

    return false;
}

//
// This function is the one to call externally to start the traversal.
// Individual functions can be initialized to 0 to skip processing of that
// type of node.  It's children will still be processed.
//
void TIntermediate::convert(TInfoSink& infoSink, bool tree)
{
    switch (version) {
    case 150:
        infoSink.debug << "#version 300 es\n";
        break;
    default:
        infoSink.debug << "#version 100\n";
    }
    infoSink.debug << "precision highp float;\n";
    infoSink.debug << "precision highp int;\n";
    if (requestedExtensions.size() > 0) {
        for (auto extIt = requestedExtensions.begin(); extIt != requestedExtensions.end(); ++extIt)
            infoSink.debug << "Requested " << *extIt << "\n";
    }

    if (xfbMode)
        infoSink.debug << "in xfb mode\n";

    if (getSubgroupUniformControlFlow())
        infoSink.debug << "subgroup_uniform_control_flow\n";

    switch (language) {
    case EShLangVertex:
        break;

    case EShLangTessControl:
        infoSink.debug << "vertices = " << vertices << "\n";

        if (inputPrimitive != ElgNone)
            infoSink.debug << "input primitive = " << TQualifier::getGeometryString(inputPrimitive) << "\n";
        if (vertexSpacing != EvsNone)
            infoSink.debug << "vertex spacing = " << TQualifier::getVertexSpacingString(vertexSpacing) << "\n";
        if (vertexOrder != EvoNone)
            infoSink.debug << "triangle order = " << TQualifier::getVertexOrderString(vertexOrder) << "\n";
        break;

    case EShLangTessEvaluation:
        infoSink.debug << "input primitive = " << TQualifier::getGeometryString(inputPrimitive) << "\n";
        infoSink.debug << "vertex spacing = " << TQualifier::getVertexSpacingString(vertexSpacing) << "\n";
        infoSink.debug << "triangle order = " << TQualifier::getVertexOrderString(vertexOrder) << "\n";
        if (pointMode)
            infoSink.debug << "using point mode\n";
        break;

    case EShLangGeometry:
        infoSink.debug << "invocations = " << invocations << "\n";
        infoSink.debug << "max_vertices = " << vertices << "\n";
        infoSink.debug << "input primitive = " << TQualifier::getGeometryString(inputPrimitive) << "\n";
        infoSink.debug << "output primitive = " << TQualifier::getGeometryString(outputPrimitive) << "\n";
        break;

    case EShLangFragment:
        if (pixelCenterInteger)
            infoSink.debug << "gl_FragCoord pixel center is integer\n";
        if (originUpperLeft)
            infoSink.debug << "gl_FragCoord origin is upper left\n";
        if (earlyFragmentTests)
            infoSink.debug << "using early_fragment_tests\n";
        if (postDepthCoverage)
            infoSink.debug << "using post_depth_coverage\n";
        if (depthLayout != EldNone)
            infoSink.debug << "using " << TQualifier::getLayoutDepthString(depthLayout) << "\n";
        if (blendEquations != 0) {
            infoSink.debug << "using";
            // blendEquations is a mask, decode it
            for (TBlendEquationShift be = (TBlendEquationShift)0; be < EBlendCount; be = (TBlendEquationShift)(be + 1)) {
                if (blendEquations & (1 << be))
                    infoSink.debug << " " << TQualifier::getBlendEquationString(be);
            }
            infoSink.debug << "\n";
        }
        if (interlockOrdering != EioNone)
            infoSink.debug << "interlock ordering = " << TQualifier::getInterlockOrderingString(interlockOrdering) << "\n";
        break;

    case EShLangMeshNV:
        infoSink.debug << "max_vertices = " << vertices << "\n";
        infoSink.debug << "max_primitives = " << primitives << "\n";
        infoSink.debug << "output primitive = " << TQualifier::getGeometryString(outputPrimitive) << "\n";
        // Fall through
    case EShLangTaskNV:
        // Fall through
    case EShLangCompute:
        infoSink.debug << "local_size = (" << localSize[0] << ", " << localSize[1] << ", " << localSize[2] << ")\n";
        {
            if (localSizeSpecId[0] != TQualifier::layoutNotSet ||
                localSizeSpecId[1] != TQualifier::layoutNotSet ||
                localSizeSpecId[2] != TQualifier::layoutNotSet) {
                infoSink.debug << "local_size ids = (" <<
                    localSizeSpecId[0] << ", " <<
                    localSizeSpecId[1] << ", " <<
                    localSizeSpecId[2] << ")\n";
            }
        }
        break;

    default:
        break;
    }

    if (treeRoot == 0 || ! tree)
        return;

    TConvertTraverser it(infoSink);
    if (getBinaryDoubleOutput())
        it.setDoubleOutput(TConvertTraverser::BinaryDoubleOutput);

    TIntermAggregate* rootSequence = treeRoot->getAsAggregate();
    TIntermAggregate* linkerObject = NULL;
    if (rootSequence != NULL) {
        TIntermSequence vec(rootSequence->getSequence());
        linkerObject = vec.back()->getAsAggregate();
        if (linkerObject != NULL) {
            int pos = 0;
            for (TIntermNode* node : vec) {
                TIntermAggregate* seq = node->getAsAggregate();
                if (seq == NULL || seq->getOp() != EOpSequence) {
                    break;
                }
                else {
                    pos++;
                }
            }
            vec.insert(vec.begin() + pos, linkerObject);
            vec.pop_back();
        }
        for (TIntermNode* node : vec) {
            node->traverse(&it);
        }
    }
}

} // end namespace glslang

#endif // !GLSLANG_WEB && !GLSLANG_ANGLE
