import * as fs from "fs";
import * as path from "path";
import * as webidl2 from "webidl2";

class AmmoJsTypeDefinitionsGenerator {
  private output: string;
  private rootTypes: webidl2.IDLRootType[];

  constructor() {
    this.output = "";
  }

  private emit(s: string) {
    this.output += s;
  }

  private emitLines(...lines: string[]) {
    this.output += lines.join("\n") + "\n";
  }

  private start(rootTypes: webidl2.IDLRootType[]) {
    this.rootTypes = rootTypes;
    this.output = "";
  }

  private unquoteString(s: string) {
    return s.replace(/^"(.+)"$/, "$1");
  }

  private isTypeClass(type: webidl2.InterfaceType | string): boolean {
    if (typeof type === "string") {
      return type !== "btIDebugDraw"; // TODO: Implement more better solution
    }

    return this.isTypeClass(type.name);
  }

  private mapType(
    typeDesc:
      | string
      | webidl2.IDLTypeDescription
      | webidl2.IDLTypeDescription[]
      | null
  ): string {
    if (typeof typeDesc === "string") {
      switch (typeDesc) {
        case "short":
        case "long":
        case "float":
        case "double":
          return "number";

        case "DOMString":
          return "string";

        default:
          return typeDesc;
      }
    }

    if (Array.isArray(typeDesc)) {
      const parent = (typeDesc[0] as any).parent as webidl2.IDLTypeDescription;
      if (parent?.generic === "FrozenArray" || parent?.generic === "sequence") {
        return this.mapType(typeDesc[0].idlType) + "[]";
      }
    } else if (typeDesc) {
      return this.mapType(typeDesc.idlType as string);
    }

    throw new Error(`Unrecognized type: ${JSON.stringify(typeDesc)}`);
  }

  private getIncludes(typeName: string): string | null {
    const includeDesc = this.rootTypes.find(
      t => t.type === "includes" && t.target === typeName
    ) as webidl2.IncludesType;
    if (includeDesc) {
      return includeDesc.includes;
    }

    return null;
  }

  private emitInterfaceMember(
    declaringType: webidl2.InterfaceType,
    declaringBaseTypeName: string | undefined,
    member: webidl2.IDLInterfaceMemberType
  ): boolean {
    if (member.type === "attribute") {
      this.emit(`\n    ${member.name}: ${this.mapType(member.idlType)};`);
      return true;
    }

    if (member.type === "operation") {
      const isCtor = member.name === declaringType.name;
      const memberName = isCtor ? "constructor" : member.name;

      if (declaringBaseTypeName) {
        const baseType = this.rootTypes.find(
          t => t.type === "interface" && t.name === declaringBaseTypeName
        ) as webidl2.InterfaceType;
        if (baseType) {
          const overriddenMember = baseType.members.find(
            m => m.type === "operation" && m.name === memberName
          ) as webidl2.OperationMemberType;
          if (
            overriddenMember &&
            overriddenMember.arguments.length !== member.arguments.length
          ) {
            this.emitInterfaceMember(baseType, undefined, overriddenMember);
          }
        }
      }

      const prefix = `    ${memberName}(`;
      const suffix =
        ")" + (isCtor ? "" : `: ${this.mapType(member.idlType)}`) + ";";

      let argStringLength = 0;
      const argList = new Array<string>();
      for (const arg of member.arguments) {
        let argName = arg.name;
        if (arg.optional) {
          argName += "?";
        }

        const s = `${argName}: ${this.mapType(arg.idlType)}`;
        argStringLength += s.length;
        argList.push(s);
      }

      this.emit("\n" + prefix);
      if (
        prefix.length +
          argStringLength +
          (argList.length - 1) * 2 +
          suffix.length >
        80
      ) {
        this.emit("\n");
        this.emit(argList.map(x => "      " + x).join(",\n"));
        this.emit("\n    ");
      } else {
        this.emit(argList.join(", "));
      }

      this.emit(suffix);

      return true;
    }

    return false;
  }

  private emitEnum(type: webidl2.EnumType) {
    this.emit(`  enum ${type.name} {\n`);
    for (let i = 0; i < type.values.length; ++i) {
      if (i > 0) {
        this.emit(",\n");
      }

      this.emit(`    ${type.values[i].value}`);
    }
    this.emit("\n  }\n\n");
  }

  private emitInterface(type: webidl2.InterfaceType) {
    const isClass = this.isTypeClass(type);
    this.emit(`  ${isClass ? "class" : "interface"} ${type.name}`);

    let baseTypeName: string | undefined;

    const includes = this.getIncludes(type.name);
    if (includes) {
      baseTypeName = includes;
    } else {
      const impl = type.extAttrs.find(x => x.name === "JSImplementation");
      if (impl) {
        baseTypeName = this.unquoteString((impl.rhs?.value as string) || "");
      }
    }

    if (baseTypeName) {
      const isBaseTypeClass = this.isTypeClass(baseTypeName);
      this.emit(
        ` ${isBaseTypeClass ? "extends" : "implements"} ${baseTypeName}`
      );
    }

    this.emit(" {");

    let numMembers = 0;
    for (const member of type.members) {
      if (this.emitInterfaceMember(type, baseTypeName, member)) {
        ++numMembers;
      }
    }

    if (numMembers > 0) {
      this.emit("\n  ");
    }

    this.emit("}\n\n");
  }

  private emitAmmoUnionType() {
    this.emit("  type Type =");
    for (const type of this.rootTypes) {
      if (type.type === "interface") {
        this.emit(`\n    | ${type.name}`);
      }
    }

    this.emit(";\n\n");
  }

  generate(webidlPath: string) {
    const idlData = fs.readFileSync(webidlPath, { encoding: "utf8" });
    const rootTypes = webidl2.parse(idlData);
    this.start(rootTypes);
    this.emitLines(
      "declare function Ammo(): Promise<void>;",
      "",
      "declare namespace Ammo {",
      "  type VoidPtr = number;",
      ""
    );

    for (const type of this.rootTypes) {
      if (type.type === "enum") {
        this.emitEnum(type);
      }

      if (type.type === "interface") {
        this.emitInterface(type);
      }
    }

    this.emitAmmoUnionType();
    this.emitLines("  function destroy(obj: Ammo.Type): void;", "}");
    return this.output;
  }
}

const generator = new AmmoJsTypeDefinitionsGenerator();
const output = generator.generate(path.resolve(__dirname, "ammo.idl"));
fs.writeFileSync(path.resolve(__dirname, "index.d.ts"), output);
