interface DisposableMujocoContact {
  geom1: number;
  geom2: number;
  delete?: () => void;
}

interface DisposableMujocoContactVector {
  get(index: number): DisposableMujocoContact | undefined;
  delete?: () => void;
}

export function consumeMujocoContact(
  getContactAt: (index: number) => DisposableMujocoContact | undefined,
  index: number,
): { geom1: number; geom2: number } | null;

export function consumeMujocoContacts(
  contactVector: DisposableMujocoContactVector,
  count: number,
): Array<{ geom1: number; geom2: number }>;
